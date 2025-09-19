#include "../include/input.h"
#include "se3_control_pkg/se3con.hpp"
#include <cassert>

// RC_Data_t implementation
RC_Data_t::RC_Data_t()
{
    rcv_stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
    last_mode = -1.0;
    last_gear = -1.0;
    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode = true;   // Default to hover mode (matches reference code)
    enter_hover_mode = false;
    is_command_mode = true; // Default to command mode (matches reference code)
    enter_command_mode = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(const px4_msgs::msg::ManualControlSetpoint::SharedPtr pMsg, TimestampManager* ts_manager)
{
    msg = *pMsg;
    
    rcv_stamp = ts_manager ? makeStampFromPX4(pMsg->timestamp, *ts_manager) : rclcpp::Clock(RCL_STEADY_TIME).now();

    ch[0] = msg.roll;      // Roll
    ch[1] = msg.pitch;     // Pitch
    ch[2] = msg.throttle;  // Thrust
    ch[3] = msg.yaw;       // Yaw

    // Apply dead zone
    for (int i = 0; i < 4; i++)
    {
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    // Extract mode switches from aux channels (normalized -1 to 1, convert to 0-1)
    mode = (msg.aux1 + 1.0) / 2.0;
    gear = (msg.aux2 + 1.0) / 2.0;
    
    // Apply low-pass filter to mode signal for stability
    filtered_mode = FILTER_ALPHA * filtered_mode + (1.0 - FILTER_ALPHA) * mode;
    
    check_validity();

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }

    // 1: judge which mode (use filtered signal for stability)
    if (last_mode < API_MODE_THRESHOLD_VALUE && filtered_mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else
        enter_hover_mode = false;

    // Use hysteresis for is_hover_mode to prevent oscillation
    if (is_hover_mode) {
        // Currently in hover mode, need to go significantly below threshold to exit
        if (filtered_mode < API_MODE_THRESHOLD_VALUE - API_MODE_HYSTERESIS)
            is_hover_mode = false;
    } else {
        // Currently not in hover mode, need to go above threshold to enter
        if (filtered_mode > API_MODE_THRESHOLD_VALUE)
            is_hover_mode = true;
    }

    // 2: judge which gear
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;
        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }
    last_mode = filtered_mode;
    last_gear = gear;
}

void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1)
    {
        // pass
    }
    else
    {
        // ROS_ERROR("[RC_Data_t] ERROR: mode=%.3f, gear=%.3f", mode, gear);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = abs(ch[0]) < 0.05 && abs(ch[1]) < 0.05 && abs(ch[2]) < 0.05 && abs(ch[3]) < 0.05;
    return centered;
}

bool RC_Data_t::is_received(const rclcpp::Time &now_time)
{
    (void)now_time; // Suppress unused parameter warning
    return true; // Temporarily disable timeout check to avoid time source conflicts
}

// LocalPosition_Data_t implementation
LocalPosition_Data_t::LocalPosition_Data_t()
{
    rcv_stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
    uav_state_ptr = nullptr;
    xy_valid = false;
    z_valid = false;
    v_xy_valid = false;
    v_z_valid = false;
}

void LocalPosition_Data_t::feed(const px4_msgs::msg::VehicleLocalPosition::SharedPtr pMsg, TimestampManager* ts_manager)
{
    rcv_stamp = ts_manager ? makeStampFromPX4(pMsg->timestamp, *ts_manager) : rclcpp::Clock(RCL_STEADY_TIME).now();
    
    // 更新有效性标志
    xy_valid = pMsg->xy_valid;
    z_valid = pMsg->z_valid;
    v_xy_valid = pMsg->v_xy_valid;
    v_z_valid = pMsg->v_z_valid;
    
    // 只有在位置数据有效时才更新UAVState
    if (uav_state_ptr != nullptr) {
        // 只有当位置有效时才更新位置
        if (xy_valid && z_valid) {
            uav_state_ptr->position << pMsg->x, pMsg->y, pMsg->z;
        }
        
        // 只有当速度有效时才更新速度
        if (v_xy_valid && v_z_valid) {
            uav_state_ptr->velocity << pMsg->vx, pMsg->vy, pMsg->vz;
        }
        
        uav_state_ptr->timestamp = ts_manager ? ts_manager->getTimeForLogging().seconds() : rcv_stamp.seconds();
    }
}

void LocalPosition_Data_t::setUAVStatePtr(se3control::UAVState* ptr) {
    uav_state_ptr = ptr;
}


// Imu_Data_t implementation
Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
    uav_state_ptr = nullptr;
}

void Imu_Data_t::feedAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr pMsg, TimestampManager* ts_manager)
{
    rcv_stamp = ts_manager ? makeStampFromPX4(pMsg->timestamp, *ts_manager) : rclcpp::Clock(RCL_STEADY_TIME).now();
    
    // 直接填充UAVState
    if (uav_state_ptr != nullptr) {
        Eigen::Quaterniond q(pMsg->q[0], pMsg->q[1], pMsg->q[2], pMsg->q[3]);
        uav_state_ptr->rotation = q.toRotationMatrix();
        uav_state_ptr->timestamp = ts_manager ? ts_manager->getTimeForLogging().seconds() : rcv_stamp.seconds();
    }
}

void Imu_Data_t::feedAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr pMsg, TimestampManager* ts_manager)
{
    rcv_stamp = ts_manager ? makeStampFromPX4(pMsg->timestamp, *ts_manager) : rclcpp::Clock(RCL_STEADY_TIME).now();
    
    // 直接填充UAVState
    if (uav_state_ptr != nullptr) {
        uav_state_ptr->angular_velocity << pMsg->xyz[0], pMsg->xyz[1], pMsg->xyz[2];
        uav_state_ptr->timestamp = ts_manager ? ts_manager->getTimeForLogging().seconds() : rcv_stamp.seconds();
    }
}

void Imu_Data_t::setUAVStatePtr(se3control::UAVState* ptr) {
    uav_state_ptr = ptr;
}

// State_Data_t implementation
State_Data_t::State_Data_t()
{
    rcv_stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
}

void State_Data_t::feed(const px4_msgs::msg::VehicleStatus::SharedPtr pMsg, TimestampManager* ts_manager)
{
    current_state = *pMsg;
    
    rcv_stamp = ts_manager ? makeStampFromPX4(pMsg->timestamp, *ts_manager) : rclcpp::Clock(RCL_STEADY_TIME).now();
}

bool State_Data_t::armed() const
{
    return current_state.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

// HoverThrustEstimate_Data_t implementation
HoverThrustEstimate_Data_t::HoverThrustEstimate_Data_t()
{
    hover_thrust = 0.0f;
    valid = false;
    rcv_stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
    uav_state_ptr = nullptr;
}

void HoverThrustEstimate_Data_t::feed(const px4_msgs::msg::HoverThrustEstimate::SharedPtr pMsg, TimestampManager* ts_manager)
{
    rcv_stamp = ts_manager ? makeStampFromPX4(pMsg->timestamp, *ts_manager) : rclcpp::Clock(RCL_STEADY_TIME).now();
    
    hover_thrust = pMsg->hover_thrust;
    valid = pMsg->valid;
    
    // 如果估计有效，直接更新UAVState中的悬停推力
    if (uav_state_ptr != nullptr && valid) {
        uav_state_ptr->hover_thrust = hover_thrust;
    }
}

void HoverThrustEstimate_Data_t::setUAVStatePtr(se3control::UAVState* ptr) {
    uav_state_ptr = ptr;
}

// TF_Data_t implementation
TF_Data_t::TF_Data_t()
{
    camera_position = Eigen::Vector3d::Zero();
    body_position = Eigen::Vector3d::Zero();
    world_position = Eigen::Vector3d::Zero();
    valid = false;
    rcv_stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
    uav_state_ptr = nullptr;
}

void TF_Data_t::feed(const tf2_msgs::msg::TFMessage::SharedPtr pMsg, TimestampManager* ts_manager)
{
    (void)ts_manager;  // 抑制未使用参数警告
    valid = false;
    
    for (const auto& transform : pMsg->transforms) {
        // 只处理AprilTag的变换 (child_frame_id == "tag")
        if (transform.child_frame_id == "base") {
            // 提取相机坐标系位置信息
            camera_position.x() = transform.transform.translation.x;
            camera_position.y() = transform.transform.translation.y;
            camera_position.z() = transform.transform.translation.z;
            
            // 直接转换到机体坐标系
            // 相机坐标系到机体坐标系的转换矩阵: (0,0,1; 1,0,0; 0,1,0)
            Eigen::Matrix3d camera_to_body;
            // camera_to_body << 0, 0, 1,    前视
            //                   1, 0, 0, 
            //                   0, 1, 0;
            // camera_to_body << 1, 0, 0,  // 左
            //                   0, 0, -1, 
            //                   0, 1, 0;
            camera_to_body  << 1, 0, 0,  // 下
                              0, 1, 0, 
                              0, 0, 1;
            body_position = camera_to_body * camera_position;
            
            // 转换到世界坐标系
            if (uav_state_ptr != nullptr) {
                // 将机体坐标系下的位置转换到世界坐标系
                Eigen::Vector3d tag_world_relative = uav_state_ptr->rotation * body_position;
                // 加上四旋翼的世界坐标系位置，得到标签的绝对世界坐标
                world_position = uav_state_ptr->position + tag_world_relative;
            } else {
                // 如果没有UAV状态，世界坐标系位置设为零
                world_position = Eigen::Vector3d::Zero();
            }
            
            // 设置时间戳
            rcv_stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            
            valid = true;
            break; // 找到tag就退出循环
        }
    }
}

void TF_Data_t::setUAVStatePtr(se3control::UAVState* ptr)
{
    uav_state_ptr = ptr;
}

bool TF_Data_t::is_received(const rclcpp::Time &now_time)
{
    bool is_timeout = (now_time - rcv_stamp).seconds() >= 0.5; // 0.5秒超时
    
    // 如果数据超时，将valid设置为false
    if (is_timeout && valid) {
        valid = false;
    }
    
    return valid && !is_timeout;
}


// ExtendedState_Data_t implementation
ExtendedState_Data_t::ExtendedState_Data_t()
{
    rcv_stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
}

void ExtendedState_Data_t::feed(const px4_msgs::msg::VehicleControlMode::SharedPtr pMsg, TimestampManager* ts_manager)
{
    extended_state = *pMsg;
    
    rcv_stamp = ts_manager ? makeStampFromPX4(pMsg->timestamp, *ts_manager) : rclcpp::Clock(RCL_STEADY_TIME).now();
}

