#include <rclcpp/rclcpp.hpp>
#include "se3_control_pkg/KF_filter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建 KF_Filter 节点
    auto kf_filter = std::make_shared<KF_Filter>();
    
    // 初始化参数（使用自身节点）
    kf_filter->initializeWithROS2Params(nullptr);
    
    RCLCPP_INFO(kf_filter->get_logger(), "KF_Filter test node started. Waiting for TF data...");
    
    // 设置初始状态（例如零状态）
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    kf_filter->setInitialState(x0, P0);
    
    // 启动节点
    rclcpp::spin(kf_filter);
    
    rclcpp::shutdown();
    return 0;
}


