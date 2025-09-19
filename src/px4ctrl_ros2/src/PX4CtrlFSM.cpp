#include "PX4CtrlFSM.h"
#include "../include/math_utils.h"
#include "se3_control_pkg/se3con.hpp"

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param) 
    : rclcpp::Node("px4ctrl_fsm"), param(param),
      state(MANUAL_CTRL), previous_state(MANUAL_CTRL), have_imu(false),
      target_position_(Eigen::Vector3d::Zero()), target_yaw_(0.0),
      target_initialized_(false), debug_print_counter_(0)
{   
    // 初始化时间戳管理器
    timestamp_manager = std::make_unique<TimestampManager>(this);
    
    // 初始化共享的UAVState对象
    current_uav_state = std::make_unique<se3control::UAVState>();
    current_uav_state->hover_thrust = param.thr_map.hover_percentage;
    
    // 初始化SE3控制器
    se3_controller = std::make_unique<se3control::SE3Controller>();
    if (!se3_controller->initialize(param)) {
        RCLCPP_ERROR(this->get_logger(), "[PX4CtrlFSM] Failed to initialize SE3 controller");
    } else {
        RCLCPP_INFO(this->get_logger(), "[PX4CtrlFSM] SE3 controller initialized successfully");
    }
    // Initialize publishers
    initializePublishers();
    // Initialize subscribers  
    initializeSubscribers();
    
    // 设置传感器数据直接填充UAVState的指针
    local_pos_data.setUAVStatePtr(current_uav_state.get());
    imu_data.setUAVStatePtr(current_uav_state.get());
    hover_thrust_data.setUAVStatePtr(current_uav_state.get());
    tf_data.setUAVStatePtr(current_uav_state.get());
    
    // 使用YAML参数文件中的控制频率
    double control_freq_hz = param.ctrl_freq_max;
    int timer_period_ms = static_cast<int>(1000.0 / control_freq_hz);
    
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&PX4CtrlFSM::process, this));  

    printControllerParameters();
    RCLCPP_INFO(this->get_logger(), "[PX4CtrlFSM] Initialization complete with %.1fHz timer (period: %dms) - ready for hover control", 
                control_freq_hz, timer_period_ms);
}

PX4CtrlFSM::~PX4CtrlFSM()
{
    // Custom destructor ensures proper cleanup of unique_ptr with forward declared type
}

void PX4CtrlFSM::initializePublishers()
{
    // 使用与稳定控制器相同的PX4 QoS设置
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);

    vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", px4_qos);
    
    offboard_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", px4_qos);
    
    thrust_setpoint_pub = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
        "/fmu/in/vehicle_thrust_setpoint", px4_qos);
    
    torque_setpoint_pub = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
        "/fmu/in/vehicle_torque_setpoint", px4_qos);
    
    // Debug publishers removed for simplification
}

void PX4CtrlFSM::initializeSubscribers()
{
    // Configure QoS for PX4 compatibility (BEST_EFFORT)
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);
        
    auto px4_high_freq_qos = rclcpp::QoS(rclcpp::KeepLast(100))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)  
        .durability(rclcpp::DurabilityPolicy::Volatile);
        

    state_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            state_data.feed(msg, timestamp_manager.get());
        });
    
   
    local_pos_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_high_freq_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            local_pos_data.feed(msg, timestamp_manager.get());
        });
    
    attitude_sub = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", px4_high_freq_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
            imu_data.feedAttitude(msg, timestamp_manager.get());
            have_imu = true;
        });

    angular_vel_sub = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        "/fmu/out/vehicle_angular_velocity", px4_high_freq_qos,
        [this](const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
            imu_data.feedAngularVelocity(msg, timestamp_manager.get());
        });
        
    rc_sub = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        "/rc_simulator/manual_control_setpoint", px4_qos,
        [this](const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) {
            rc_data.feed(msg, timestamp_manager.get());
        });
        
    hover_thrust_sub = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
        "/fmu/out/hover_thrust_estimate", px4_qos,
        [this](const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg) {
            hover_thrust_data.feed(msg, timestamp_manager.get());
            RCLCPP_DEBUG(this->get_logger(), "Hover thrust updated: %.4f (valid: %s)", 
                        msg->hover_thrust, msg->valid ? "true" : "false");
        });
        
    // 订阅tf话题
    auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(50))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile);
        
    tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf_qos,
        std::bind(&PX4CtrlFSM::tfCallback, this, std::placeholders::_1));
}

void PX4CtrlFSM::process()
{
    // 使用时间戳管理器获取最佳时间戳（带外推的微秒时间戳）
    auto now_time = timestamp_manager->getTimeForLogging(); // 仅用于日志显示
    se3control::UAVCommand desired_command;
    // 检查TF数据是否有效（包括超时检查）
    bool tf_valid_and_received = tf_is_received(now_time);
    if (tf_data.valid != tf_valid_and_received) {
        tf_data.valid = tf_valid_and_received;
    }

    // 调用调试信息打印函数
    printDebugInfo(now_time);
    publish_offboard_control_mode();
    switch(state) {
        case MANUAL_CTRL:
        {
            if (rc_data.enter_hover_mode) 
            {
                // 检查安全条件
                if (current_uav_state->velocity.norm() > 3.0)
                {
                    RCLCPP_ERROR(this->get_logger(), "[FSM] 拒绝进入悬停模式 - 速度过大: %.1fm/s", current_uav_state->velocity.norm());
                }
                if (!local_pos_is_received(now_time))
                {
                    RCLCPP_ERROR(this->get_logger(), "[FSM] 拒绝进入悬停模式,无位置信息");
                }
                if (!imu_is_received(now_time))
                {
                    RCLCPP_ERROR(this->get_logger(), "[FSM] 拒绝进入悬停模式,无IMU数据");
                }
                
                // 检查当前位置数据是否有效
                if (current_uav_state->position.norm() < 0.01) {
                    RCLCPP_ERROR(this->get_logger(), "[FSM] 拒绝进入悬停模式 - 位置数据无效 [%.3f,%.3f,%.3f]", 
                                current_uav_state->position(0), current_uav_state->position(1), current_uav_state->position(2));
                }
                
                // 切换到悬停模式
                state = AUTO_HOVER;
                set_hov_with_local_pos();
                // 启动offboard模式
                toggle_offboard_mode(true);
           
            }
            return;
        }
            
        case AUTO_HOVER:
        {
            if (!rc_data.is_hover_mode||!local_pos_is_received(now_time)||!imu_is_received(now_time)) 
            {
                state = MANUAL_CTRL;
                toggle_offboard_mode(false);
                RCLCPP_WARN(this->get_logger(), "[FSM] HOVER --> MANUAL");
                return;
            }
            else if (rc_data.enter_command_mode)
            {
                if (state_data.current_state.nav_state == 14 && tf_data.valid)
                {
                    state = CMD_CTRL;
                    target_initialized_ = false; // 进入CMD_CTRL时重置目标位置标志
                    RCLCPP_INFO(this->get_logger(), "[FSM] HOVER --> CMD_CTRL");
                }
            }
            
            // 始终生成悬停控制命令
            desired_command = createUAVCommand(target_position_, 
                                     Eigen::Vector3d::Zero(), 
                                     Eigen::Vector3d::Zero(), 
                                     target_yaw_);
            break;
        }

        case CMD_CTRL:
        {
            
            if (!rc_data.is_hover_mode||!local_pos_is_received(now_time)||!imu_is_received(now_time)) {
                state = MANUAL_CTRL;
                toggle_offboard_mode(false);
                RCLCPP_WARN(this->get_logger(), "[FSM] CMD_CTRL --> MANUAL");
                return;
            }
            else if (!tf_data.valid || !rc_data.is_command_mode)
            {
                state = AUTO_HOVER;
                target_initialized_ = false; // 退出CMD_CTRL时重置目标位置标志
                set_hov_with_local_pos();
                RCLCPP_WARN(this->get_logger(), "[FSM] CMD_CTRL --> AUTO_HOVER");
            }
            else
            {
                // 只在刚进入CMD_CTRL模式时设置一次目标位置
                if (!target_initialized_) {
                    setFixedTargetPosition();
                    target_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "[CMD_CTRL] 目标位置已锁定为: [%.3f,%.3f,%.3f], yaw: %.1f°", 
                               target_position_(0), target_position_(1), target_position_(2),
                               target_yaw_ * 180.0 / M_PI);
                }
                
                desired_command = createUAVCommand(target_position_, 
                                                 Eigen::Vector3d::Zero(), 
                                                 Eigen::Vector3d::Zero(), 
                                                 target_yaw_);
            }
            break;
        }
    }
  
    se3control::UAVState current_state = getCurrentUAVState(now_time.seconds());
    se3control::ControlOutput control_output = se3_controller->computeControl(current_state, desired_command);
    publish_thrust_torque_setpoints(control_output, now_time);

    // Removed takeoff/land data clearing as requested
}

se3control::UAVState PX4CtrlFSM::getCurrentUAVState(double timestamp_seconds)
{
    // 设置悬停油门百分比
    double hover_throttle_percentage = param.thr_map.hover_percentage;
    current_uav_state->hover_thrust = hover_throttle_percentage;
    current_uav_state->timestamp = timestamp_seconds;
    
    return *current_uav_state;
}

void PX4CtrlFSM::publish_thrust_torque_setpoints(const se3control::ControlOutput &out, const rclcpp::Time &)
{
    // 使用PX4时间戳管理器获取PX4兼容的时间戳
    uint64_t timestamp = timestamp_manager->getCurrentTimestampUs();
  
    px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
    thrust_msg.timestamp = timestamp;
    thrust_msg.xyz[0] = 0.0f;  // X推力
    thrust_msg.xyz[1] = 0.0f;  // Y推力  
    thrust_msg.xyz[2] = static_cast<float>(out.thrust);  // Z推力 (控制器已提供负值表示向上)
    thrust_setpoint_pub->publish(thrust_msg);
    
    // 发布力矩设定点
    px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
    torque_msg.timestamp = timestamp;
    torque_msg.xyz[0] = static_cast<float>(out.moment(0));  // Roll力矩
    torque_msg.xyz[1] = static_cast<float>(out.moment(1));  // Pitch力矩
    torque_msg.xyz[2] = static_cast<float>(out.moment(2));  // Yaw力矩
    torque_setpoint_pub->publish(torque_msg);
}

void PX4CtrlFSM::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = timestamp_manager->getCurrentTimestampUs();
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = true;  // 使用推力和力矩控制
    msg.direct_actuator = false;
    
    offboard_mode_pub->publish(msg);
}

void PX4CtrlFSM::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand vehicle_command{};
    
    // 使用时间戳管理器获取最佳时间戳
    uint64_t timestamp_us = timestamp_manager->getCurrentTimestampUs();
    vehicle_command.timestamp = timestamp_us;
    vehicle_command.param1 = param1;
    vehicle_command.param2 = param2;
    vehicle_command.command = command;
    vehicle_command.target_system = 1;
    vehicle_command.target_component = 1;
    vehicle_command.source_system = 1;
    vehicle_command.source_component = 1;
    vehicle_command.from_external = true;
    vehicle_command_pub->publish(vehicle_command);
//   - 1.0 - MANUAL（手动模式）
//   - 2.0 - ALTITUDE（定高模式）
//   - 3.0 - POSITION（定点模式）
//   - 4.0 - AUTO（自动模式）
//   - 5.0 - ACRO（特技模式）
//   - 6.0 - OFFBOARD（离线模式）
//   - 7.0 - STABILIZED（自稳模式）
}

void PX4CtrlFSM::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "[PX4CtrlFSM] Arm command sent");
}

void PX4CtrlFSM::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "[PX4CtrlFSM] Disarm command sent");
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
    if (on_off) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // OFFBOARD mode
        RCLCPP_INFO(this->get_logger(), "[PX4CtrlFSM] Offboard mode enabled");
    } else {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 7.0); // MANUAL mode
        RCLCPP_INFO(this->get_logger(), "[PX4CtrlFSM] Manual mode enabled");
    }
    return true;
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
    if (arm) {
        // Check if already armed
        if (state_data.armed()) {
            RCLCPP_DEBUG(this->get_logger(), "[PX4CtrlFSM] Drone is already armed");
            return true;
        }
        // 检查解锁前置条件
        if (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL) {
            RCLCPP_INFO(this->get_logger(), "\033[33m[ARM] Arming in MANUAL mode, this is normal for initial arming\033[0m");
        }
        
        this->arm();
        RCLCPP_INFO(this->get_logger(), "\033[36m[ARM] Arm command sent, waiting for PX4 response...\033[0m");
    } else {
        // Check if already disarmed
        if (!state_data.armed()) {
            RCLCPP_DEBUG(this->get_logger(), "[PX4CtrlFSM] Drone is already disarmed");
            return true;
        }
        this->disarm();
        RCLCPP_INFO(this->get_logger(), "\033[36m[DISARM] Disarm command sent\033[0m");
    }
    
    // Note: PX4 command success cannot be immediately verified as it's asynchronous
    // The actual arm/disarm status should be checked via state_data.armed() in subsequent calls
    return true;
}

bool PX4CtrlFSM::imu_is_received(const rclcpp::Time &now_time)
{
    // 检查IMU数据接收时间戳，超时时间设为0.5秒
    const double imu_timeout = 0.5;  // 秒
    // 使用传入的时间戳以保持一致性，如果为零则使用当前时间
    auto reference_time = (now_time.nanoseconds() == 0) ? rclcpp::Clock(RCL_STEADY_TIME).now() : now_time;
    return (reference_time - imu_data.rcv_stamp).seconds() < imu_timeout;
}

bool PX4CtrlFSM::local_pos_is_received(const rclcpp::Time &now_time)
{
    // 检查位置数据接收时间戳，超时时间设为0.5秒
    const double pos_timeout = 0.5;  // 秒，位置数据更新频率通常较高
    // 使用传入的时间戳以保持一致性，如果为零则使用当前时间
    auto reference_time = (now_time.nanoseconds() == 0) ? rclcpp::Clock(RCL_STEADY_TIME).now() : now_time;
    
    bool time_valid = (reference_time - local_pos_data.rcv_stamp).seconds() < pos_timeout;
    bool position_valid = local_pos_data.isPositionValid();
    
    return time_valid && position_valid;
}

bool PX4CtrlFSM::rc_is_received(const rclcpp::Time &now_time)
{
    (void)now_time;  // 抑制未使用参数警告
    // 检查RC遥控器数据接收时间戳，超时时间设为1.0秒
    const double rc_timeout = 1.0;  // 秒，RC数据更新频率较低，给更长的超时时间
    // 使用简单的时间差计算避免时间源冲突
    auto current_steady_time = rclcpp::Clock(RCL_STEADY_TIME).now();
    return (current_steady_time - rc_data.rcv_stamp).seconds() < rc_timeout;
}

bool PX4CtrlFSM::tf_is_received(const rclcpp::Time &now_time)
{
    (void)now_time;  // 抑制未使用参数警告
    // 使用简单的时间差计算避免时间源冲突
    auto current_steady_time = rclcpp::Clock(RCL_STEADY_TIME).now();
    return tf_data.valid && ((current_steady_time - tf_data.rcv_stamp).seconds() < 0.5);
}

void PX4CtrlFSM::set_hov_with_local_pos()
{
    // 将当前位置设置为悬停目标位置
    target_position_ = current_uav_state->position;
    
    // 正确计算偏航角：从旋转矩阵提取欧拉角 (ZYX顺序)
    // 使用Eigen的eulerAngles方法：eulerAngles(2,1,0) = (yaw, pitch, roll)
    auto euler = current_uav_state->rotation.eulerAngles(2, 1, 0);
    target_yaw_ = euler(0);  // 提取偏航角 (Z轴旋转)
    
    RCLCPP_INFO(this->get_logger(), "[HOVER] 悬停点设置为当前位置: [%.3f, %.3f, %.3f], yaw: %.1f°",
               target_position_(0), target_position_(1), target_position_(2), 
               target_yaw_ * 180.0 / M_PI);
}

void PX4CtrlFSM::printControllerParameters()
{
    RCLCPP_INFO(this->get_logger(), "=== CONTROLLER PARAMETERS (from YAML) ===");
    
    // 基础参数
    RCLCPP_INFO(this->get_logger(), "Basic Parameters:");
    RCLCPP_INFO(this->get_logger(), "  ctrl_freq_max: %.1f Hz", param.ctrl_freq_max);
    RCLCPP_INFO(this->get_logger(), "  max_manual_vel: %.1f m/s", param.max_manual_vel);
    
    // 推力映射参数
    RCLCPP_INFO(this->get_logger(), "Thrust Mapping:");
    RCLCPP_INFO(this->get_logger(), "  hover_percentage: %.3f", param.thr_map.hover_percentage);
    
    // SE3位置控制增益
    RCLCPP_INFO(this->get_logger(), "SE3 Position Gains:");
    RCLCPP_INFO(this->get_logger(), "  Kp: [%.1f, %.1f, %.1f]", param.se3.Kp_x, param.se3.Kp_y, param.se3.Kp_z);
    RCLCPP_INFO(this->get_logger(), "  Kv: [%.1f, %.1f, %.1f]", param.se3.Kv_x, param.se3.Kv_y, param.se3.Kv_z);
    RCLCPP_INFO(this->get_logger(), "  Kvi: [%.2f, %.2f, %.2f]", param.se3.Kvi_x, param.se3.Kvi_y, param.se3.Kvi_z);
    
    // SE3姿态控制增益
    RCLCPP_INFO(this->get_logger(), "SE3 Attitude Gains:");
    RCLCPP_INFO(this->get_logger(), "  Kr: [%.1f, %.1f, %.1f]", param.se3.Kr_x, param.se3.Kr_y, param.se3.Kr_z);
    RCLCPP_INFO(this->get_logger(), "  Kw: [%.2f, %.2f, %.2f]", param.se3.Kw_x, param.se3.Kw_y, param.se3.Kw_z);
    RCLCPP_INFO(this->get_logger(), "  Kri: [%.2f, %.2f, %.2f]", param.se3.Kri_x, param.se3.Kri_y, param.se3.Kri_z);
    
    // 物理参数
    RCLCPP_INFO(this->get_logger(), "Physical Parameters:");
    RCLCPP_INFO(this->get_logger(), "  mass: %.1f kg", param.se3.mass);
    RCLCPP_INFO(this->get_logger(), "  gravity: %.2f m/s²", param.se3.gravity);
    RCLCPP_INFO(this->get_logger(), "  Inertia: [%.3f, %.3f, %.3f] kg⋅m²", param.se3.Ixx, param.se3.Iyy, param.se3.Izz);
    
    // 控制限制
    RCLCPP_INFO(this->get_logger(), "Control Limits:");
    RCLCPP_INFO(this->get_logger(), "  max_thrust: %.3f", param.se3.max_thrust);
    RCLCPP_INFO(this->get_logger(), "  min_thrust: %.3f", param.se3.min_thrust);
    RCLCPP_INFO(this->get_logger(), "  max_torque: %.2f", param.se3.max_torque);
    
    // 积分控制
    RCLCPP_INFO(this->get_logger(), "Integral Control:");
    RCLCPP_INFO(this->get_logger(), "  use_integral: %s", param.se3.use_integral ? "true" : "false");
    
    // RC反向参数
    RCLCPP_INFO(this->get_logger(), "RC Reverse Settings:");
    RCLCPP_INFO(this->get_logger(), "  roll: %s, pitch: %s, yaw: %s, throttle: %s", 
               param.rc_reverse.roll ? "true" : "false",
               param.rc_reverse.pitch ? "true" : "false", 
               param.rc_reverse.yaw ? "true" : "false",
               param.rc_reverse.throttle ? "true" : "false");
    
    RCLCPP_INFO(this->get_logger(), "=========================================");
}

se3control::UAVCommand PX4CtrlFSM::createUAVCommand(const Eigen::Vector3d& position,
                                                   const Eigen::Vector3d& velocity,
                                                   const Eigen::Vector3d& acceleration,
                                                   double yaw)
{
    se3control::UAVCommand desired_command;
    // 设置位置、速度和加速度
    desired_command.position = position;
    desired_command.velocity = velocity;
    desired_command.acceleration = acceleration;
    
    // 根据偏航角计算期望的机体x轴方向
    desired_command.b1d = Eigen::Vector3d(cos(yaw), sin(yaw), 0.0);
    desired_command.yaw_desired = yaw;
    
    return desired_command;
}

void PX4CtrlFSM::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    // 直接处理TF数据，坐标转换已在feed函数中完成
    tf_data.feed(msg, timestamp_manager.get());
}

void PX4CtrlFSM::calculateTargetPositionFromAprilTag()
{
    if (!tf_data.valid) {
        return; // 如果TF数据无效，不更新目标位置
    }
    
    // 期望的AprilTag在机体坐标系中的位置 (相对于无人机的期望位置)
    // 例如：AprilTag在机体前方2米，高度保持当前，左右居中
    Eigen::Vector3d desired_tag_pos_body(2.0, 0.0, 0.0);  // x=前方2米, y=左右居中, z=高度相同
    
    // 当前AprilTag在机体坐标系的实际位置
    Eigen::Vector3d current_tag_pos_body = tf_data.body_position;
    
    // 计算位置误差 (机体坐标系)
    Eigen::Vector3d position_error_body = current_tag_pos_body - desired_tag_pos_body;
    
    // 将位置误差从机体坐标系转换到全局坐标系
    Eigen::Vector3d position_error_global = current_uav_state->rotation * position_error_body;
    
    // 更新目标位置：当前位置 - 位置误差 = 期望位置
    target_position_ = current_uav_state->position - position_error_global;
    
    // 保持当前偏航角
    auto euler = current_uav_state->rotation.eulerAngles(2, 1, 0);
    target_yaw_ = euler(0);
    
    RCLCPP_DEBUG(this->get_logger(), "[AprilTag Control] Tag body pos: [%.3f,%.3f,%.3f], Target pos: [%.3f,%.3f,%.3f]",
                current_tag_pos_body(0), current_tag_pos_body(1), current_tag_pos_body(2),
                target_position_(0), target_position_(1), target_position_(2));
}

void PX4CtrlFSM::setFixedTargetPosition()
{
    // 检查TF数据是否有效
    if (tf_data.valid) {
        // 额外检查AprilTag位置是否合理（防止异常数据导致飞到危险位置）
        double tag_distance = tf_data.world_position.norm();
        if (tag_distance > 0.1 && tag_distance < 50.0) {  // 标签距离在0.1m-50m之间才认为有效
            // 使用AprilTag的世界坐标位置，但增加安全偏移
            target_position_(0) = tf_data.world_position(0);  // X: 标签位置向后0.3米
            target_position_(1) = tf_data.world_position(1);        // Y: 与标签相同
            target_position_(2) = tf_data.world_position(2)-1;        // Z: 与标签相同高度
            
            // 使用当前偏航角作为目标偏航角（进入CMD模式时的偏航角）
            // 正确计算偏航角：从旋转矩阵提取欧拉角
            auto euler = current_uav_state->rotation.eulerAngles(2, 1, 0);
            target_yaw_ = euler(0);  // 提取偏航角
            
            RCLCPP_DEBUG(this->get_logger(), "[AprilTag Target] Using AprilTag position: tag[%.2f,%.2f,%.2f] -> target[%.2f,%.2f,%.2f], yaw: %.1f deg",
                        tf_data.world_position(0), tf_data.world_position(1), tf_data.world_position(2),
                        target_position_(0), target_position_(1), target_position_(2),
                        target_yaw_ * 180.0 / M_PI);
        } else {
            RCLCPP_WARN(this->get_logger(), "[AprilTag Target] 标签位置异常，距离: %.2f，保持当前目标位置", tag_distance);
        }
    } else {
        RCLCPP_DEBUG(this->get_logger(), "[AprilTag Target] TF数据无效，保持当前目标位置");
    }
}

void PX4CtrlFSM::printRCDebugInfo(const rclcpp::Time &now_time)
{
    // 检查遥控器数据是否接收到
    bool rc_received = rc_is_received(now_time);
    
    // 打印遥控器基础数据
    RCLCPP_INFO(this->get_logger(), 
        "[RC] 接收状态:%s 通道值:[%.3f,%.3f,%.3f,%.3f]",
        rc_received ? "OK" : "TIMEOUT",
        rc_data.ch[0], rc_data.ch[1], rc_data.ch[2], rc_data.ch[3]);
    
    // 打印遥控器模式和档位信息
    RCLCPP_INFO(this->get_logger(), 
        "[RC] Mode:%.3f(滤波:%.3f) Gear:%.3f 悬停模式:%s 指令模式:%s",
        rc_data.mode, rc_data.filtered_mode, rc_data.gear,
        rc_data.is_hover_mode ? "ON" : "OFF",
        rc_data.is_command_mode ? "ON" : "OFF");
    
    // 打印遥控器触发状态
    RCLCPP_INFO(this->get_logger(), 
        "[RC] 进入悬停:%s 进入指令:%s 居中状态:%s",
        rc_data.enter_hover_mode ? "是" : "否",
        rc_data.enter_command_mode ? "是" : "否", 
        rc_data.check_centered() ? "居中" : "偏离");
        
    // 打印原始消息数据
    RCLCPP_DEBUG(this->get_logger(), 
        "[RC_RAW] roll:%.3f pitch:%.3f yaw:%.3f throttle:%.3f aux1:%.3f aux2:%.3f",
        rc_data.msg.roll, rc_data.msg.pitch, rc_data.msg.yaw, rc_data.msg.throttle,
        rc_data.msg.aux1, rc_data.msg.aux2);
}

void PX4CtrlFSM::printDroneStateDebugInfo(const rclcpp::Time &now_time)
{
    // 检查数据接收状态
    bool pos_received = local_pos_is_received(now_time);
    bool imu_received = imu_is_received(now_time);
    
    // 打印位置信息
    RCLCPP_INFO(this->get_logger(), 
        "[DRONE] 位置接收:%s 坐标:[%.3f,%.3f,%.3f] 速度:[%.3f,%.3f,%.3f]",
        pos_received ? "OK" : "TIMEOUT",
        current_uav_state->position(0), current_uav_state->position(1), current_uav_state->position(2),
        current_uav_state->velocity(0), current_uav_state->velocity(1), current_uav_state->velocity(2));
    
    // 从旋转矩阵提取欧拉角
    auto euler = current_uav_state->rotation.eulerAngles(2, 1, 0);
    double roll = euler(2) * 180.0 / M_PI;
    double pitch = euler(1) * 180.0 / M_PI; 
    double yaw = euler(0) * 180.0 / M_PI;
    
    // 打印姿态信息
    RCLCPP_INFO(this->get_logger(), 
        "[DRONE] IMU接收:%s 姿态(度):[R%.1f P%.1f Y%.1f] 角速度:[%.3f,%.3f,%.3f]",
        imu_received ? "OK" : "TIMEOUT",
        roll, pitch, yaw,
        current_uav_state->angular_velocity(0), current_uav_state->angular_velocity(1), current_uav_state->angular_velocity(2));
    
    // 打印目标位置和偏航
    RCLCPP_INFO(this->get_logger(), 
        "[DRONE] 目标位置:[%.3f,%.3f,%.3f] 目标偏航:%.1f° 悬停推力:%.3f",
        target_position_(0), target_position_(1), target_position_(2),
        target_yaw_ * 180.0 / M_PI, current_uav_state->hover_thrust);
}

void PX4CtrlFSM::printFSMDebugInfo(const rclcpp::Time &now_time)
{
    // FSM状态字符串映射
    const char* state_str;
    const char* prev_state_str;
    bool state_changed = (state != previous_state);
    
    switch(state) {
        case MANUAL_CTRL: state_str = "MANUAL_CTRL"; break;
        case AUTO_HOVER: state_str = "AUTO_HOVER"; break;
        case CMD_CTRL: state_str = "CMD_CTRL"; break;
        default: state_str = "UNKNOWN"; break;
    }
    
    switch(previous_state) {
        case MANUAL_CTRL: prev_state_str = "MANUAL_CTRL"; break;
        case AUTO_HOVER: prev_state_str = "AUTO_HOVER"; break;
        case CMD_CTRL: prev_state_str = "CMD_CTRL"; break;
        default: prev_state_str = "UNKNOWN"; break;
    }
    
    // 如果状态发生变化，使用彩色高亮显示
    if (state_changed) {
        RCLCPP_INFO(this->get_logger(), 
            "\033[1;32m[FSM] 状态切换: %s → %s\033[0m 解锁状态:%s 飞行模式:NAV_%d 目标初始化:%s",
            prev_state_str, state_str,
            state_data.armed() ? "ARMED" : "DISARMED",
            state_data.current_state.nav_state,
            target_initialized_ ? "是" : "否");
        
        // 更新previous_state用于下次比较
        previous_state = state;
    } else {
        // 正常状态显示
        RCLCPP_INFO(this->get_logger(), 
            "[FSM] 当前状态:%s 解锁状态:%s 飞行模式:NAV_%d 目标初始化:%s",
            state_str,
            state_data.armed() ? "ARMED" : "DISARMED",
            state_data.current_state.nav_state,
            target_initialized_ ? "是" : "否");
    }
    
    // 打印各数据源接收状态
    RCLCPP_INFO(this->get_logger(), 
        "[FSM] 数据源状态 - RC:%s POS:%s IMU:%s TF:%s",
        rc_is_received(now_time) ? "OK" : "LOST",
        local_pos_is_received(now_time) ? "OK" : "LOST",
        imu_is_received(now_time) ? "OK" : "LOST",
        tf_is_received(now_time) ? "OK" : "LOST");
}

void PX4CtrlFSM::printAprilTagDebugInfo(const rclcpp::Time &now_time)
{
    (void)now_time;  // 抑制未使用参数警告
    // 打印AprilTag检测状态
    RCLCPP_INFO(this->get_logger(), 
        "[APRILTAG] TF有效:%s 接收时间:%.3fs前",
        tf_data.valid ? "是" : "否",
        (rclcpp::Clock(RCL_STEADY_TIME).now() - tf_data.rcv_stamp).seconds());
    
    if (tf_data.valid) {
        // 打印世界坐标系中的标签位置
        RCLCPP_INFO(this->get_logger(), 
            "[APRILTAG] 世界坐标:[%.3f,%.3f,%.3f] 距离:%.3fm",
            tf_data.world_position(0), tf_data.world_position(1), tf_data.world_position(2),
            tf_data.world_position.norm());
        
        // 打印机体坐标系中的标签位置  
        RCLCPP_INFO(this->get_logger(), 
            "[APRILTAG] 机体坐标:[%.3f,%.3f,%.3f] (前-右-下)",
            tf_data.body_position(0), tf_data.body_position(1), tf_data.body_position(2));
    } else {
        RCLCPP_DEBUG(this->get_logger(), "[APRILTAG] 未检测到标签或数据超时");
    }
}

void PX4CtrlFSM::printSE3ControllerDebugInfo(const rclcpp::Time &now_time)
{
    (void)now_time;  // 抑制未使用参数警告
    if (state == MANUAL_CTRL) {
        RCLCPP_DEBUG(this->get_logger(), "[SE3] 手动模式，控制器未激活");
        return;
    }
    
    // 打印SE3控制器输入（期望状态）
    RCLCPP_INFO(this->get_logger(), 
        "[SE3_INPUT] 期望位置:[%.3f,%.3f,%.3f] 期望偏航:%.1f°",
        target_position_(0), target_position_(1), target_position_(2),
        target_yaw_ * 180.0 / M_PI);
    
    // 计算位置误差
    Eigen::Vector3d position_error = target_position_ - current_uav_state->position;
    auto euler = current_uav_state->rotation.eulerAngles(2, 1, 0);
    double yaw_error = (target_yaw_ - euler(0)) * 180.0 / M_PI;
    
    // 处理偏航角误差的周期性（-180到180度）
    while (yaw_error > 180.0) yaw_error -= 360.0;
    while (yaw_error < -180.0) yaw_error += 360.0;
    
    RCLCPP_INFO(this->get_logger(), 
        "[SE3_ERROR] 位置误差:[%.3f,%.3f,%.3f] 距离:%.3fm 偏航误差:%.1f°",
        position_error(0), position_error(1), position_error(2),
        position_error.norm(), yaw_error);
    
    // 注意：实际的控制输出（推力和力矩）在publish_thrust_torque_setpoints中已发布
    // 这里打印概要信息
    RCLCPP_DEBUG(this->get_logger(), 
        "[SE3_OUTPUT] 控制输出已发布 - 推力+力矩模式");
}

void PX4CtrlFSM::printDebugInfo(const rclcpp::Time &now_time)
{
    // 每200次调用打印一次（1秒间隔，因为控制频率是200Hz）
    if (++debug_print_counter_ < 200) {
        return;
    }
    debug_print_counter_ = 0; // 重置计数器
    
    
    printFSMDebugInfo(now_time);
    printAprilTagDebugInfo(now_time);
}


