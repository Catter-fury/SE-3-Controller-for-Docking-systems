#include <rclcpp/rclcpp.hpp>
#include "../include/PX4CtrlFSM.h"
#include <signal.h>
#include <thread>
#include <chrono>

// 全局FSM节点指针，用于信号处理
std::shared_ptr<PX4CtrlFSM> global_fsm_node = nullptr;

void mySigintHandler(int)
{
    RCLCPP_INFO(rclcpp::get_logger("px4ctrl"), "[PX4Ctrl] Interrupt signal received - exiting...");
    
    // 等待一小段时间确保日志输出
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    signal(SIGINT, mySigintHandler);
    
    // Load parameters
    Parameter_t param;
    auto temp_node = std::make_shared<rclcpp::Node>("px4ctrl_fsm");  
    param.config_from_ros_handle(temp_node);

    // Create FSM node with loaded parameters
    auto fsm_node = std::make_shared<PX4CtrlFSM>(param);
    global_fsm_node = fsm_node;  // 设置全局指针用于信号处理

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rclcpp::spin(fsm_node);  

    global_fsm_node = nullptr;

    rclcpp::shutdown();
    return 0;
}
