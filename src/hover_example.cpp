#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "se3_control_pkg/se3con.hpp"
#include "input.h"
#include "PX4CtrlParam.h"
#include <Eigen/Dense>

using namespace se3control;

class HoverExampleNode : public rclcpp::Node
{
public:
    HoverExampleNode() : Node("hover_example_node")
    {
        // 初始化目标位置和偏航角（将从当前位置和姿态设置）
        target_position_ = Eigen::Vector3d::Zero();
        target_yaw_ = 0;
        hover_thrust_ = 0.47;  // 悬停油门默认值（将从配置文件覆盖）
        
        RCLCPP_INFO(this->get_logger(), "=== SE3 Control Hover Example ===");
        RCLCPP_INFO(this->get_logger(), "Target hover position: [%.2f, %.2f, %.2f] m (NED)", 
                    target_position_(0), target_position_(1), target_position_(2));
        RCLCPP_INFO(this->get_logger(), "Target yaw angle: %.1f degrees", target_yaw_ * 180.0 / M_PI);
        
        // 初始化发布者和订阅者
        initializePublishersAndSubscribers();
        
        // 初始化SE3控制器
        initializeSE3Controller();
        
        // 创建控制循环定时器 (50Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&HoverExampleNode::controlLoop, this));
            
        RCLCPP_INFO(this->get_logger(), "Hover example node initialized. Waiting for sensor data...");
    }

private:
    // 目标状态
    Eigen::Vector3d target_position_;
    double target_yaw_;
    double hover_thrust_;
    bool target_initialized_ = false;
    
    // SE3控制器
    SE3Controller se3_controller_;
    bool controller_initialized_ = false;
    
    // 传感器数据
    LocalPosition_Data_t local_pos_data_;
    Imu_Data_t imu_data_;
    bool have_position_ = false;
    bool have_attitude_ = false;
    bool have_angular_velocity_ = false;
    
    // 发布者
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    // 订阅者
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_vel_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // 控制状态
    rclcpp::Time start_time_;
    bool control_active_ = false;
    
    void initializePublishersAndSubscribers()
    {
        // PX4 QoS设置
        auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);
            
        // 发布者
        thrust_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/in/vehicle_thrust_setpoint", px4_qos);
        torque_pub_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
            "/fmu/in/vehicle_torque_setpoint", px4_qos);
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", px4_qos);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", px4_qos);
            
        // 订阅者
        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", px4_qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                local_pos_data_.feed(msg);
                have_position_ = true;
            });
            
        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", px4_qos,
            [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                imu_data_.feedAttitude(msg);
                have_attitude_ = true;
            });
            
        angular_vel_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", px4_qos,
            [this](const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
                imu_data_.feedAngularVelocity(msg);
                have_angular_velocity_ = true;
            });
    }
    
    void initializeSE3Controller()
    {
        // 使用临时节点读取参数，避免shared_from_this()问题
        Parameter_t param;
        auto temp_node = std::make_shared<rclcpp::Node>("px4ctrl_fsm");
        param.config_from_ros_handle(temp_node);
        
        // 从配置文件读取悬停推力参数
        hover_thrust_ = param.thr_map.hover_percentage;
        
        // 初始化SE3控制器
        if (!se3_controller_.initialize(param)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize SE3 controller!");
            return;
        }
        
        controller_initialized_ = true;
        start_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "SE3 controller initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Hover thrust from config: %.3f", hover_thrust_);
        RCLCPP_INFO(this->get_logger(), "Control gains - Kp:[%.1f,%.1f,%.1f] Kv:[%.1f,%.1f,%.1f]", 
                   param.se3.Kp_x, param.se3.Kp_y, param.se3.Kp_z,
                   param.se3.Kv_x, param.se3.Kv_y, param.se3.Kv_z);
        RCLCPP_INFO(this->get_logger(), "Attitude gains - Kr:[%.1f,%.1f,%.1f] Kw:[%.2f,%.2f,%.2f]", 
                   param.se3.Kr_x, param.se3.Kr_y, param.se3.Kr_z,
                   param.se3.Kw_x, param.se3.Kw_y, param.se3.Kw_z);
    }
    
    void controlLoop()
    {
        if (!controller_initialized_) {
            return;
        }
        
        auto current_time = this->get_clock()->now();
        double elapsed_time = (current_time - start_time_).seconds();
        
        // 始终发布offboard控制模式
        publishOffboardControlMode();
        
        // 检查传感器数据是否有效
        if (!have_position_ || !have_attitude_ || !have_angular_velocity_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "Waiting for sensor data - Pos:%s Att:%s AngVel:%s", 
                                 have_position_ ? "✓" : "✗", 
                                 have_attitude_ ? "✓" : "✗",
                                 have_angular_velocity_ ? "✓" : "✗");
            return;
        }
        
        // 首次设置目标位置和偏航角为当前状态
        if (!target_initialized_) {
            target_position_ = local_pos_data_.p;  // 使用当前位置作为悬停目标
        
            // 从当前姿态矩阵提取偏航角
            auto euler = imu_data_.q.toRotationMatrix().eulerAngles(2, 1, 0);
            target_yaw_ = euler(0);  // 使用当前偏航角
            
            target_initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Target initialized to current state:");
            RCLCPP_INFO(this->get_logger(), "Position: [%.2f, %.2f, %.2f] m", 
                       target_position_(0), target_position_(1), target_position_(2));
            RCLCPP_INFO(this->get_logger(), "Yaw: %.1f degrees", target_yaw_ * 180.0 / M_PI);
        }
        
        // // 延迟3秒后开始控制，确保系统稳定
        // if (!control_active_) {
        //     if (elapsed_time > 0.1) {
        //         control_active_ = true;
        //         RCLCPP_INFO(this->get_logger(), "Starting hover control at current position!");
        //         publishArmCommand();
        //         publishOffboardModeCommand();
        //     } else {
        //         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //                             "Initializing... Starting control in %.1f seconds", 
        //                             3.0 - elapsed_time);
        //         return;
        //     }
        // }
        if(!control_active_){
        control_active_=true;
        publishArmCommand();
        publishOffboardModeCommand();
        }
        se3control::UAVState current_state = getCurrentUAVState(current_time);
        
        // 创建期望指令 - 定点悬停  
        se3control::UAVCommand desired_command = createHoverDesiredCommand();
        
        // 计算SE3控制输出
        ControlOutput control_output = se3_controller_.computeControl(current_state, desired_command);
        
        // 发布控制指令
        publishControlSetpoints(control_output);
        
        // 定期打印状态信息
        logHoverStatus(current_state, control_output, elapsed_time);
    }
    
    se3control::UAVCommand createHoverDesiredCommand()
    {
        se3control::UAVCommand cmd;
        
        // 期望位置 - 固定悬停点
        cmd.position = target_position_;
        
        // 期望速度 - 悬停时为零
        cmd.velocity = Eigen::Vector3d::Zero();
        
        // 期望加速度 - 悬停时为零  
        cmd.acceleration = Eigen::Vector3d::Zero();
        
        // 期望机体x轴方向 - 直接从偏航角计算
        cmd.b1d = Eigen::Vector3d(cos(target_yaw_), sin(target_yaw_), 0.0);
        
        return cmd;
    }
    
    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = true;  // 使用推力和力矩控制
        msg.direct_actuator = false;
        
        offboard_mode_pub_->publish(msg);
    }
    
    void publishControlSetpoints(const ControlOutput& output)
    {
        auto timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        // 发布推力设定点
        px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
        thrust_msg.timestamp = timestamp;
        thrust_msg.xyz[0] = 0.0f;  // X推力
        thrust_msg.xyz[1] = 0.0f;  // Y推力  
        thrust_msg.xyz[2] = static_cast<float>(output.thrust);  // Z推力 (控制器已提供负值表示向上)
        thrust_pub_->publish(thrust_msg);
        
        // 发布力矩设定点
        px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
        torque_msg.timestamp = timestamp;
        torque_msg.xyz[0] = static_cast<float>(output.moment(0));  // Roll力矩
        torque_msg.xyz[1] = static_cast<float>(output.moment(1));  // Pitch力矩
        torque_msg.xyz[2] = static_cast<float>(output.moment(2));  // Yaw力矩
        torque_pub_->publish(torque_msg);
    }
    
    void publishArmCommand()
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.param1 = 1.0f;  // 1 = ARM, 0 = DISARM
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        RCLCPP_INFO(this->get_logger(), 
                       "arm enable");
        vehicle_command_pub_->publish(msg);
    }
    
    void publishOffboardModeCommand()
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.param1 = 1.0f;  // Main mode
        msg.param2 = 6.0f;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
           RCLCPP_INFO(this->get_logger(), 
                       "offboard enable");
    }
    
    se3control::UAVState getCurrentUAVState(const rclcpp::Time& current_time)
    {
        se3control::UAVState current_state;
        current_state.position = local_pos_data_.p;
        current_state.velocity = local_pos_data_.v;
        current_state.rotation = imu_data_.q.toRotationMatrix();
        current_state.angular_velocity = imu_data_.w;
        current_state.hover_thrust = hover_thrust_;
        current_state.timestamp = current_time.seconds();
        return current_state;
    }
    
    void h()
    {
        // 简单的悬停函数，参考PX4CtrlFSM.cpp中的set_hov_with_local_pos函数
        // 设置目标位置为当前位置，实现定点悬停
        if (have_position_ && have_attitude_) {
            target_position_ = local_pos_data_.p;
            
            // 从当前姿态矩阵提取偏航角
            auto euler = imu_data_.q.toRotationMatrix().eulerAngles(2, 1, 0);
            target_yaw_ = euler(0);
            
            RCLCPP_INFO(this->get_logger(), "h() function called - Setting hover target to current position");
            RCLCPP_INFO(this->get_logger(), "New target: [%.2f, %.2f, %.2f] m, yaw: %.1f°", 
                       target_position_(0), target_position_(1), target_position_(2), 
                       target_yaw_ * 180.0 / M_PI);
        } else {
            RCLCPP_WARN(this->get_logger(), "h() function called but sensor data not available");
        }
    }
    
    void logHoverStatus(const UAVState& current_state, const ControlOutput& control_output, double elapsed_time)
    {
        static int log_counter = 0;
        if (++log_counter >= 50) {  // 每1秒打印一次状态 (50Hz控制频率)
            // 计算位置误差
            Eigen::Vector3d position_error = current_state.position - target_position_;
            double distance_error = position_error.norm();
            double velocity_magnitude = current_state.velocity.norm();
            
            // 转换旋转矩阵到欧拉角显示
            auto euler = current_state.rotation.eulerAngles(2, 1, 0);  // ZYX顺序
            double yaw_deg = euler(0) * 180.0 / M_PI;
            double pitch_deg = euler(1) * 180.0 / M_PI; 
            double roll_deg = euler(2) * 180.0 / M_PI;
            
            // 角度归一化到[-180, 180]
            auto normalize_angle = [](double& angle) {
                while (angle > 180.0) angle -= 360.0;
                while (angle < -180.0) angle += 360.0;
            };
            normalize_angle(roll_deg);
            normalize_angle(pitch_deg);
            normalize_angle(yaw_deg);
            
            // 控制状态评估
            std::string status = "STABILIZING";
            if (distance_error < 0.1 && velocity_magnitude < 0.05) {
                status = "HOVERING";
            } else if (distance_error < 0.5) {
                status = "CONVERGING";
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "=== HOVER STATUS (T+%.1fs) ===", elapsed_time);
            RCLCPP_INFO(this->get_logger(), 
                       "Position: [%.2f, %.2f, %.2f] | Target: [%.2f, %.2f, %.2f] | Error: %.3fm", 
                       current_state.position(0), current_state.position(1), current_state.position(2),
                       target_position_(0), target_position_(1), target_position_(2), distance_error);
            RCLCPP_INFO(this->get_logger(), 
                       "Velocity: [%.2f, %.2f, %.2f] | Magnitude: %.2fm/s | Status: %s",
                       current_state.velocity(0), current_state.velocity(1), current_state.velocity(2),
                       velocity_magnitude, status.c_str());
            RCLCPP_INFO(this->get_logger(), 
                       "Attitude: Roll=%.1f° Pitch=%.1f° Yaw=%.1f° | Target Yaw=%.1f°",
                       roll_deg, pitch_deg, yaw_deg, target_yaw_ * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), 
                       "Control: Thrust=%.3f | Torque=[%.3f, %.3f, %.3f]",
                       control_output.thrust, control_output.moment(0), 
                       control_output.moment(1), control_output.moment(2));
            RCLCPP_INFO(this->get_logger(), "================================");
            
            log_counter = 0;
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<HoverExampleNode>();
    
    RCLCPP_INFO(rclcpp::get_logger("hover_example"), 
               "SE3 Control Hover Example - Press Ctrl+C to stop");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}