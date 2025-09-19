#include"se3_control_pkg/KF_filter.hpp"
#include"se3_control_pkg/se3con.hpp"  // 包含完整的UAVState定义
#include <iostream>
#include <cmath>
#include <stdexcept>

KF_Filter::KF_Filter() : 
    rclcpp::Node("kf_filter_node"),
    x_(VectorXd::Zero(6)),
    P_(MatrixXd::Identity(6, 6)),
    A_(MatrixXd::Identity(6, 6)),
    Q_(MatrixXd::Identity(6, 6) * 0.01),
    R_(MatrixXd::Identity(6, 6) * 0.1),
    dt_(0.03),
    initialized_(false),
    last_update_time_(rclcpp::Clock().now()) 
{
    // 初始化时间戳管理器
    timestamp_manager_ = std::make_unique<TimestampManager>(this);
    
    // 创建 TF 订阅者
    auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile);
        
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf_qos,
        std::bind(&KF_Filter::tfCallback, this, std::placeholders::_1));
        
    RCLCPP_INFO(this->get_logger(), "[KF_Filter] Initialized with TF subscription");
}

//初始化参数   
void KF_Filter::initializeWithROS2Params(rclcpp::Node::SharedPtr node) {
    // 使用传入的节点或自身节点加载参数
    if (node) {
        param_.config_from_ros_handle(node);
        RCLCPP_INFO(node->get_logger(), "[KF_Filter] Loading parameters from external node");
    } else {
        // 只有在已经初始化为shared_ptr时才能调用shared_from_this()
        try {
            param_.config_from_ros_handle(shared_from_this());
            RCLCPP_INFO(this->get_logger(), "[KF_Filter] Loading parameters from self");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[KF_Filter] Failed to load parameters: %s", e.what());
            // 使用默认参数
            param_.ctrl_freq_max = 50.0;  // 默认50Hz
            param_.kf.R_pos = 0.1;        // 默认位置测量噪声
        }
    }
    
    dt_ = 1.0 / param_.ctrl_freq_max;

    // 设置测量噪声协方差（3x3 位置测量噪声）
    R_ = MatrixXd::Identity(3, 3) * param_.kf.R_pos;

    initialized_ = true;
    
    if (node) {
        RCLCPP_INFO(node->get_logger(), "[KF_Filter] Parameters initialized, dt=%.3f", dt_);
    } else {
        RCLCPP_INFO(this->get_logger(), "[KF_Filter] Parameters initialized, dt=%.3f", dt_);
    }
}

//预测
void KF_Filter::predict() {
    if (!initialized_) {
        throw std::runtime_error("KF在预测前未初始化");
    }
    
    // 计算实际时间步长
    auto now = rclcpp::Clock().now();
    double dt = (now - last_update_time_).seconds();
    if (dt > 0) {
        dt_ = dt;
        last_update_time_ = now;
    }
    
    // 更新状态转移矩阵
    A_.block<3,3>(0,3) = Matrix3d::Identity() * dt_;
    
    // 状态预测
    x_ = A_ * x_;
    
    // 协方差预测
    P_ = A_ * P_ * A_.transpose() + Q_;
}

// 使用TF数据进行校正
void KF_Filter::correctWithTF(const TF_Data_t& tf_data) {
    if (!initialized_) {
        throw std::runtime_error("KF在校正前未初始化");
    }
    
    if (!tf_data.valid) {
        return;  // 无效数据不处理
    }

     // 提取位置观测值
    Vector3d z = tf_data.world_position;

    // 观测矩阵 (只观测位置)
    MatrixXd H = MatrixXd::Zero(3, 6);
    H.block<3,3>(0,0) = Matrix3d::Identity();

    // 计算卡尔曼增益
    MatrixXd S = H * P_ * H.transpose() + R_;
    MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // 状态更新
    VectorXd innovation = z - H * x_;
    x_ = x_ + K * innovation;
    
    // 协方差更新
    MatrixXd I = MatrixXd::Identity(6, 6);
    P_ = (I - K * H) * P_;
    
    // 确保协方差矩阵保持对称
    P_ = 0.5 * (P_ + P_.transpose());
}

// 获取UAV状态（用于控制器）
se3control::UAVState KF_Filter::getUAVState() const {
    se3control::UAVState state;
    if (x_.size() >= 6) {
        state.position = x_.segment<3>(0);
        state.velocity = x_.segment<3>(3);
    }
    state.timestamp = last_update_time_.seconds();
 
    return state;
}

// TF 回调函数实现
void KF_Filter::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    // 处理 TF 数据
    tf_data_.feed(msg, timestamp_manager_.get());
    
    // 如果数据有效，使用它进行卡尔曼滤波校正
    if (tf_data_.valid) {
        try {
            correctWithTF(tf_data_);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "[KF_Filter] TF correction failed: %s", e.what());
        }
    }
}

// 其他成员函数实现...
VectorXd KF_Filter::getState() const {
    return x_;
}

MatrixXd KF_Filter::getCovariance() const {
    return P_;
}

void KF_Filter::setProcessNoise(const MatrixXd& Q) {
    if (Q.rows() != 6 || Q.cols() != 6) {
        throw std::invalid_argument("过程噪声矩阵必须为6x6");
    }
    Q_ = Q;
}

void KF_Filter::setMeasurementNoise(const MatrixXd& R) {
    if (R.rows() != 3 || R.cols() != 3) {
        throw std::invalid_argument("位置测量噪声矩阵为3x3");
    }
    R_ = R;
}

void KF_Filter::reset() {
    x_.setZero();
    P_ = MatrixXd::Identity(6, 6);
    last_update_time_ = rclcpp::Clock().now();
}

void KF_Filter::setInitialState(const VectorXd& x0, const MatrixXd& P0) {
    if (x0.size() != 6) {
        throw std::invalid_argument("Initial state must be 6-dimensional");
    }
    if (P0.rows() != 6 || P0.cols() != 6) {
        throw std::invalid_argument("Initial covariance must be 6x6");
    }
    
    x_ = x0;
    P_ = P0;
    last_update_time_ = rclcpp::Clock().now();
    initialized_ = true;
    
}
