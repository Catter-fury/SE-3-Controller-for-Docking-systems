#ifndef __INPUT_H
#define __INPUT_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

// ROS2 standard messages
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

// PX4 messages for ROS2
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>

// TF2 messages
#include <tf2_msgs/msg/tf_message.hpp>

#include "PX4CtrlParam.h"
#include "timestamp_manager.h"
#include "math_utils.h"

// Forward declaration for se3control::UAVState
namespace se3control {
    struct UAVState;
}

// Utility function for centralized PX4 timestamp handling
inline rclcpp::Time makeStampFromPX4(uint64_t px4_ts, TimestampManager& tm) {
    if (px4_ts > 0) {
        tm.updatePX4Timestamp(px4_ts);
        uint64_t unified_timestamp_us = tm.getCurrentTimestampUs();
        return rclcpp::Time(unified_timestamp_us * 1000, RCL_STEADY_TIME);
    } else {
        return rclcpp::Clock(RCL_STEADY_TIME).now();
    }
}


struct TakeoffLand {
    rclcpp::Time header_stamp;
    uint8_t takeoff_land_cmd;
    bool triggered;  // 标志是否接收到新的takeoff/land命令
    static constexpr uint8_t TAKEOFF = 1;
    static constexpr uint8_t LAND = 2;
    static constexpr uint8_t NO_REQUEST = 0;
    
    TakeoffLand() : takeoff_land_cmd(NO_REQUEST), triggered(false) {}
};

class RC_Data_t
{
public:
  double mode;
  double gear;
  double last_mode;
  double last_gear;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  
  // Signal filtering for stability
  double filtered_mode{0.0};
  static constexpr double FILTER_ALPHA = 0.8;  // Low-pass filter coefficient
  double ch[4];

  px4_msgs::msg::ManualControlSetpoint msg;
  rclcpp::Time rcv_stamp;

  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75; // Match ROS1 threshold
  static constexpr double API_MODE_HYSTERESIS = 0.15;      // Larger hysteresis for stability
  static constexpr double DEAD_ZONE = 0.25;

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(const px4_msgs::msg::ManualControlSetpoint::SharedPtr pMsg, TimestampManager* ts_manager = nullptr);
  bool is_received(const rclcpp::Time &now_time);
};

class LocalPosition_Data_t
{
public:
  rclcpp::Time rcv_stamp;
  se3control::UAVState* uav_state_ptr;
  bool xy_valid;
  bool z_valid;
  bool v_xy_valid;
  bool v_z_valid;

  LocalPosition_Data_t();
  void feed(const px4_msgs::msg::VehicleLocalPosition::SharedPtr pMsg, TimestampManager* ts_manager = nullptr);
  void setUAVStatePtr(se3control::UAVState* ptr);
  bool isPositionValid() const { return xy_valid && z_valid; }
  bool isVelocityValid() const { return v_xy_valid && v_z_valid; }
};

class Imu_Data_t
{
public:
  rclcpp::Time rcv_stamp;
  se3control::UAVState* uav_state_ptr;

  Imu_Data_t();
  void feedAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr pMsg, TimestampManager* ts_manager = nullptr);
  void feedAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr pMsg, TimestampManager* ts_manager = nullptr);
  void setUAVStatePtr(se3control::UAVState* ptr);
};


class State_Data_t
{
public:
  px4_msgs::msg::VehicleStatus current_state;
  rclcpp::Time rcv_stamp;
  
  State_Data_t();
  void feed(const px4_msgs::msg::VehicleStatus::SharedPtr pMsg, TimestampManager* ts_manager = nullptr);
  
  bool armed() const;
};

class HoverThrustEstimate_Data_t
{
public:
  float hover_thrust;
  bool valid;
  rclcpp::Time rcv_stamp;
  se3control::UAVState* uav_state_ptr;
  
  HoverThrustEstimate_Data_t();
  void feed(const px4_msgs::msg::HoverThrustEstimate::SharedPtr pMsg, TimestampManager* ts_manager = nullptr);
  void setUAVStatePtr(se3control::UAVState* ptr);
};

class TF_Data_t


{
public:
  Eigen::Vector3d camera_position;   // AprilTag在相机坐标系的位置 (x, y, z)
  Eigen::Vector3d body_position;     // AprilTag在机体坐标系的位置 (x, y, z) - 直接转换得到
  Eigen::Vector3d world_position;    // AprilTag在世界坐标系的位置 (x, y, z) - 绝对位置
  bool valid;
  rclcpp::Time rcv_stamp;
  se3control::UAVState* uav_state_ptr;
  
  TF_Data_t();
  void feed(const tf2_msgs::msg::TFMessage::SharedPtr pMsg, TimestampManager* ts_manager = nullptr);
  bool is_received(const rclcpp::Time &now_time);
  void setUAVStatePtr(se3control::UAVState* ptr);
};

class ExtendedState_Data_t
{
public:
  px4_msgs::msg::VehicleControlMode extended_state;
  rclcpp::Time rcv_stamp;
  
  ExtendedState_Data_t();
  void feed(const px4_msgs::msg::VehicleControlMode::SharedPtr pMsg, TimestampManager* ts_manager = nullptr);
};


// 期望状态结构 - 直接适配SE3控制器UAVCommand接口
struct Desired_State_t
{
    Eigen::Vector3d p;        // 期望位置
    Eigen::Vector3d v;        // 期望速度 
    Eigen::Vector3d a;        // 期望加速度
    Eigen::Vector3d j;        // 期望加加速度
    Eigen::Vector3d b1d;      // 期望机体x轴方向向量 (直接用于SE3控制器)
    double yaw;               // 期望偏航角（用于计算b1d和调试显示）
    double yaw_rate;          // 期望偏航角速度

    Desired_State_t()
        : p(Eigen::Vector3d::Zero()),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()),
          b1d(Eigen::Vector3d(1.0, 0.0, 0.0)),  // 默认朝向x轴正方向
          yaw(0.0),
          yaw_rate(0.0) {}

    // 移除了基于传感器数据的构造函数，因为数据结构已简化
    
    // 从偏航角更新b1d向量
    void update_b1d_from_yaw() {
        b1d = Eigen::Vector3d(cos(yaw), sin(yaw), 0.0);
    }
    
    // 设置偏航角并自动更新b1d
    void set_yaw(double new_yaw) {
        yaw = new_yaw;
        update_b1d_from_yaw();
    }
          
private:
    double get_yaw_from_quaternion(const Eigen::Quaterniond& q) {
        return quatToYaw(q);
    }
};

// Legacy structure kept only for test node compatibility
struct Controller_Output_t
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d thrust;     // 推力向量 [Fx, Fy, Fz]
    Eigen::Vector3d torque;     // 力矩向量 [Mx, My, Mz]
    
    Controller_Output_t() : 
        thrust(Eigen::Vector3d::Zero()),
        torque(Eigen::Vector3d::Zero()) {}
};

#endif