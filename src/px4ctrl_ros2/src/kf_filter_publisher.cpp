#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "se3_control_pkg/KF_filter.hpp"
#include "se3_control_pkg/se3con.hpp"  // 包含完整的UAVState定义

/**
 * KF_Filter_Publisher 节点
 * 功能：订阅TF数据，使用卡尔曼滤波器处理，然后发布滤波后的状态
 */
class KF_Filter_Publisher : public rclcpp::Node {
public:
    KF_Filter_Publisher() : Node("kf_filter_publisher") {
        // 创建 KF_Filter 实例
        kf_filter_ = std::make_shared<KF_Filter>();
        
        // 初始化 KF_Filter 参数（传入当前节点指针）
        kf_filter_->initializeWithROS2Params(shared_from_this());
        
        // 设置初始状态
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        kf_filter_->setInitialState(x0, P0);
        
        // 创建发布者
        setupPublishers();
        
        // 创建定时器，定期发布滤波后的数据
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz 发布频率
            std::bind(&KF_Filter_Publisher::publishFilteredData, this));
            
        RCLCPP_INFO(this->get_logger(), "[KF_Filter_Publisher] Node started");
        RCLCPP_INFO(this->get_logger(), "Publishing topics:");
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/position");
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/velocity"); 
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/odometry");
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/state_vector");
        RCLCPP_INFO(this->get_logger(), "  - /kf_filtered/covariance");
    }

private:
    void setupPublishers() {
        // 滤波后的位置 (geometry_msgs/PointStamped)
        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/kf_filtered/position", 10);
            
        // 滤波后的速度 (geometry_msgs/TwistStamped)
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/kf_filtered/velocity", 10);
            
        // 滤波后的里程计信息 (nav_msgs/Odometry)
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/kf_filtered/odometry", 10);
            
        // 完整状态向量 (std_msgs/Float64MultiArray)
        state_vector_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/kf_filtered/state_vector", 10);
            
        // 协方差矩阵 (std_msgs/Float64MultiArray) 
        covariance_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/kf_filtered/covariance", 10);
    }
    
    void publishFilteredData() {
        try {
            // 获取滤波后的UAV状态
            auto uav_state = kf_filter_->getUAVState();
            auto state_vector = kf_filter_->getState();
            auto covariance = kf_filter_->getCovariance();
            
            // 调试：检查数据有效性
            static int debug_counter = 0;
            debug_counter++;
            if (debug_counter % 100 == 1) {  // 每5秒打印一次调试信息
                RCLCPP_INFO(this->get_logger(), 
                    "[Debug] State: pos=[%.3f,%.3f,%.3f], vel=[%.3f,%.3f,%.3f], timestamp=%.3f", 
                    uav_state.position.x(), uav_state.position.y(), uav_state.position.z(),
                    uav_state.velocity.x(), uav_state.velocity.y(), uav_state.velocity.z(),
                    uav_state.timestamp);
                    
                if (state_vector.size() >= 6) {
                    RCLCPP_INFO(this->get_logger(), 
                        "[Debug] State vector: [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                        state_vector(0), state_vector(1), state_vector(2),
                        state_vector(3), state_vector(4), state_vector(5));
                }
            }
            
            auto now = this->get_clock()->now();
            
            // 发布位置信息
            publishPosition(uav_state, now);
            
            // 发布速度信息
            publishVelocity(uav_state, now);
            
            // 发布里程计信息
            publishOdometry(uav_state, now);
            
            // 发布状态向量
            publishStateVector(state_vector, now);
            
            // 发布协方差矩阵
            publishCovariance(covariance, now);
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "[KF_Filter_Publisher] Failed to publish data: %s", e.what());
        }
    }
    
    void publishPosition(const se3control::UAVState& state, const rclcpp::Time& timestamp) {
        geometry_msgs::msg::PointStamped pos_msg;
        pos_msg.header.stamp = timestamp;
        pos_msg.header.frame_id = "world";
        pos_msg.point.x = state.position.x();
        pos_msg.point.y = state.position.y();
        pos_msg.point.z = state.position.z();
        
        position_pub_->publish(pos_msg);
        
        // 调试：每50个消息打印一次发布确认
        static int pos_counter = 0;
        pos_counter++;
        if (pos_counter % 50 == 1) {
            RCLCPP_INFO(this->get_logger(), 
                "[Publish] Position #%d: [%.3f,%.3f,%.3f]", 
                pos_counter, pos_msg.point.x, pos_msg.point.y, pos_msg.point.z);
        }
    }
    
    void publishVelocity(const se3control::UAVState& state, const rclcpp::Time& timestamp) {
        geometry_msgs::msg::TwistStamped vel_msg;
        vel_msg.header.stamp = timestamp;
        vel_msg.header.frame_id = "world";
        vel_msg.twist.linear.x = state.velocity.x();
        vel_msg.twist.linear.y = state.velocity.y();
        vel_msg.twist.linear.z = state.velocity.z();
        // 角速度设为0（KF只处理位置和线速度）
        vel_msg.twist.angular.x = 0.0;
        vel_msg.twist.angular.y = 0.0;
        vel_msg.twist.angular.z = 0.0;
        
        velocity_pub_->publish(vel_msg);
    }
    
    void publishOdometry(const se3control::UAVState& state, const rclcpp::Time& timestamp) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "base_link";
        
        // 位置
        odom_msg.pose.pose.position.x = state.position.x();
        odom_msg.pose.pose.position.y = state.position.y();
        odom_msg.pose.pose.position.z = state.position.z();
        
        // 姿态（暂时设为单位四元数，因为KF不处理姿态）
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;
        
        // 速度
        odom_msg.twist.twist.linear.x = state.velocity.x();
        odom_msg.twist.twist.linear.y = state.velocity.y();
        odom_msg.twist.twist.linear.z = state.velocity.z();
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
        
        // 设置协方差矩阵（从KF获取）
        auto cov = kf_filter_->getCovariance();
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                if (i < 3 && j < 3) {
                    // 位置协方差
                    odom_msg.pose.covariance[i * 6 + j] = cov(i, j);
                } else if (i >= 3 && j >= 3) {
                    // 速度协方差
                    odom_msg.twist.covariance[(i-3) * 6 + (j-3)] = cov(i, j);
                }
            }
        }
        
        odometry_pub_->publish(odom_msg);
    }
    
    void publishStateVector(const Eigen::VectorXd& state, const rclcpp::Time& /*timestamp*/) {
        std_msgs::msg::Float64MultiArray state_msg;
        state_msg.layout.dim.resize(1);
        state_msg.layout.dim[0].label = "state";
        state_msg.layout.dim[0].size = state.size();
        state_msg.layout.dim[0].stride = state.size();
        
        state_msg.data.resize(state.size());
        for (int i = 0; i < state.size(); ++i) {
            state_msg.data[i] = state(i);
        }
        
        state_vector_pub_->publish(state_msg);
    }
    
    void publishCovariance(const Eigen::MatrixXd& covariance, const rclcpp::Time& /*timestamp*/) {
        std_msgs::msg::Float64MultiArray cov_msg;
        cov_msg.layout.dim.resize(2);
        cov_msg.layout.dim[0].label = "rows";
        cov_msg.layout.dim[0].size = covariance.rows();
        cov_msg.layout.dim[0].stride = covariance.rows() * covariance.cols();
        cov_msg.layout.dim[1].label = "cols";
        cov_msg.layout.dim[1].size = covariance.cols();
        cov_msg.layout.dim[1].stride = covariance.cols();
        
        cov_msg.data.resize(covariance.rows() * covariance.cols());
        for (int i = 0; i < covariance.rows(); ++i) {
            for (int j = 0; j < covariance.cols(); ++j) {
                cov_msg.data[i * covariance.cols() + j] = covariance(i, j);
            }
        }
        
        covariance_pub_->publish(cov_msg);
    }

private:
    // KF Filter 实例
    std::shared_ptr<KF_Filter> kf_filter_;
    
    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_vector_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr covariance_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<KF_Filter_Publisher>();
    
    RCLCPP_INFO(node->get_logger(), "KF Filter Publisher started. Press Ctrl+C to stop.");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
