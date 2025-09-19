#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <Eigen/Dense>
#include "input.h"
#include "timestamp_manager.h"

/**
 * 简化版本的 KF_Filter_Publisher
 * 直接在单个节点中处理TF订阅、卡尔曼滤波和数据发布
 */
class SimpleKFPublisher : public rclcpp::Node {
public:
    SimpleKFPublisher() : Node("simple_kf_publisher") {
        // 延迟初始化参数，避免shared_from_this()问题
        // param_.config_from_ros_handle(shared_from_this());
        
        // 初始化卡尔曼滤波器状态
        initializeKF();
        
        // 初始化时间戳管理器
        timestamp_manager_ = std::make_unique<TimestampManager>(this);
        
        // 创建TF订阅者
        auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);
            
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", tf_qos,
            std::bind(&SimpleKFPublisher::tfCallback, this, std::placeholders::_1));
        
        // 创建发布者
        setupPublishers();
        
        // 创建定时器，定期发布滤波后的数据
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz
            std::bind(&SimpleKFPublisher::publishFilteredData, this));
            
        RCLCPP_INFO(this->get_logger(), "[SimpleKFPublisher] Node started with TF subscription and KF filtering");
        RCLCPP_INFO(this->get_logger(), "Publishing topics:");
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/position");
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/velocity");
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/odometry");
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/state_vector");
    }

private:
    void initializeKF() {
        // 初始化6维状态向量 [x, y, z, vx, vy, vz]
        x_ = Eigen::VectorXd::Zero(6);
        
        // 初始化协方差矩阵
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        
        // 使用默认参数
        double ctrl_freq_max = 50.0;  // 默认50Hz
        double kf_R_pos = 0.1;        // 默认位置测量噪声
        
        // 状态转移矩阵（恒速模型）
        A_ = Eigen::MatrixXd::Identity(6, 6);
        double dt = 1.0 / ctrl_freq_max;
        A_.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        
        // 过程噪声协方差
        Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
        
        // 测量噪声协方差（只观测位置）
        R_ = Eigen::MatrixXd::Identity(3, 3) * kf_R_pos;
        
        // 观测矩阵（只观测位置）
        H_ = Eigen::MatrixXd::Zero(3, 6);
        H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        
        kf_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), "[SimpleKFPublisher] KF initialized with dt=%.3f", dt);
    }
    
    void setupPublishers() {
        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/kf_filtered/position", 10);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/kf_filtered/velocity", 10);
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/kf_filtered/odometry", 10);
        state_vector_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/kf_filtered/state_vector", 10);
    }
    
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        // 处理TF数据
        tf_data_.feed(msg, timestamp_manager_.get());
        
        // 如果数据有效，进行卡尔曼滤波校正
        if (tf_data_.valid && kf_initialized_) {
            try {
                // 预测步骤
                predict();
                
                // 校正步骤
                correctWithTF();
                
                tf_update_count_++;
                if (tf_update_count_ % 10 == 1) {
                    RCLCPP_INFO(this->get_logger(), 
                        "[TF Update #%d] World position: [%.3f,%.3f,%.3f]", 
                        tf_update_count_,
                        tf_data_.world_position.x(), 
                        tf_data_.world_position.y(), 
                        tf_data_.world_position.z());
                }
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "[SimpleKFPublisher] KF update failed: %s", e.what());
            }
        }
    }
    
    void predict() {
        // 状态预测：x = A * x
        x_ = A_ * x_;
        
        // 协方差预测：P = A * P * A^T + Q
        P_ = A_ * P_ * A_.transpose() + Q_;
    }
    
    void correctWithTF() {
        // 获取观测值（TF世界位置）
        Eigen::VectorXd z(3);
        z << tf_data_.world_position.x(), tf_data_.world_position.y(), tf_data_.world_position.z();
        
        // 计算创新
        Eigen::VectorXd innovation = z - H_ * x_;
        
        // 计算创新协方差
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        
        // 计算卡尔曼增益
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        
        // 状态更新
        x_ = x_ + K * innovation;
        
        // 协方差更新（Joseph形式）
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_ = (I - K * H_) * P_;
        
        // 确保协方差矩阵保持对称
        P_ = 0.5 * (P_ + P_.transpose());
    }
    
    void publishFilteredData() {
        if (!kf_initialized_) {
            return;
        }
        
        auto now = this->get_clock()->now();
        
        // 发布位置
        publishPosition(now);
        
        // 发布速度
        publishVelocity(now);
        
        // 发布里程计
        publishOdometry(now);
        
        // 发布状态向量
        publishStateVector(now);
        
        // 调试信息
        publish_count_++;
        if (publish_count_ % 100 == 1) {
            RCLCPP_INFO(this->get_logger(), 
                "[Publish #%d] KF State: pos=[%.3f,%.3f,%.3f], vel=[%.3f,%.3f,%.3f]",
                publish_count_,
                x_(0), x_(1), x_(2), x_(3), x_(4), x_(5));
        }
    }
    
    void publishPosition(const rclcpp::Time& now) {
        geometry_msgs::msg::PointStamped pos_msg;
        pos_msg.header.stamp = now;
        pos_msg.header.frame_id = "world";
        pos_msg.point.x = x_(0);
        pos_msg.point.y = x_(1);
        pos_msg.point.z = x_(2);
        
        position_pub_->publish(pos_msg);
    }
    
    void publishVelocity(const rclcpp::Time& now) {
        geometry_msgs::msg::TwistStamped vel_msg;
        vel_msg.header.stamp = now;
        vel_msg.header.frame_id = "world";
        vel_msg.twist.linear.x = x_(3);
        vel_msg.twist.linear.y = x_(4);
        vel_msg.twist.linear.z = x_(5);
        vel_msg.twist.angular.x = 0.0;
        vel_msg.twist.angular.y = 0.0;
        vel_msg.twist.angular.z = 0.0;
        
        velocity_pub_->publish(vel_msg);
    }
    
    void publishOdometry(const rclcpp::Time& now) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "base_link";
        
        // 位置
        odom_msg.pose.pose.position.x = x_(0);
        odom_msg.pose.pose.position.y = x_(1);
        odom_msg.pose.pose.position.z = x_(2);
        
        // 姿态（单位四元数）
        odom_msg.pose.pose.orientation.w = 1.0;
        
        // 速度
        odom_msg.twist.twist.linear.x = x_(3);
        odom_msg.twist.twist.linear.y = x_(4);
        odom_msg.twist.twist.linear.z = x_(5);
        
        // 协方差（简化版本）
        for (int i = 0; i < 36; ++i) {
            odom_msg.pose.covariance[i] = 0.0;
            odom_msg.twist.covariance[i] = 0.0;
        }
        // 只设置对角线元素
        for (int i = 0; i < 3; ++i) {
            odom_msg.pose.covariance[i * 6 + i] = P_(i, i);  // 位置协方差
            odom_msg.twist.covariance[i * 6 + i] = P_(i+3, i+3);  // 速度协方差
        }
        
        odometry_pub_->publish(odom_msg);
    }
    
    void publishStateVector(const rclcpp::Time& now) {
        std_msgs::msg::Float64MultiArray state_msg;
        state_msg.layout.dim.resize(1);
        state_msg.layout.dim[0].label = "state";
        state_msg.layout.dim[0].size = 6;
        state_msg.layout.dim[0].stride = 6;
        
        state_msg.data.resize(6);
        for (int i = 0; i < 6; ++i) {
            state_msg.data[i] = x_(i);
        }
        
        state_vector_pub_->publish(state_msg);
    }

private:
    // 参数（注释掉，使用默认值）
    // Parameter_t param_;
    
    // 卡尔曼滤波器状态
    Eigen::VectorXd x_;      // 状态向量
    Eigen::MatrixXd P_;      // 协方差矩阵
    Eigen::MatrixXd A_;      // 状态转移矩阵
    Eigen::MatrixXd Q_;      // 过程噪声协方差
    Eigen::MatrixXd R_;      // 测量噪声协方差
    Eigen::MatrixXd H_;      // 观测矩阵
    bool kf_initialized_;
    
    // TF数据处理
    TF_Data_t tf_data_;
    std::unique_ptr<TimestampManager> timestamp_manager_;
    
    // ROS2组件
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_vector_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // 计数器
    int tf_update_count_ = 0;
    int publish_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleKFPublisher>();
    
    RCLCPP_INFO(node->get_logger(), "Simple KF Publisher started. Press Ctrl+C to stop.");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
