#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <iomanip>
#include <memory>

// PX4-ROS2 message headers
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Custom headers
#include "input.h"
// Forward declarations to avoid circular includes
namespace se3control {
    class SE3Controller;
    struct UAVState;
    struct ControlOutput;
    struct UAVCommand;
}
#include "timestamp_manager.h"


class PX4CtrlFSM : public rclcpp::Node
{
public:
    PX4CtrlFSM(Parameter_t &param);
    ~PX4CtrlFSM(); // Custom destructor needed for unique_ptr with forward declaration

    enum FSM_State
    {
        MANUAL_CTRL,
        AUTO_HOVER,
        CMD_CTRL
    };

    // FSM methods
    void process();
    void process1();
    // Hover control functions - public for external access
    void start_hover_at_current_position();
    void stop_hover_and_land();
    void set_hov_with_local_pos();
    
    // Debug functions
    void printDebugInfo(const rclcpp::Time &now_time);
    void printRCDebugInfo(const rclcpp::Time &now_time);
    void printDroneStateDebugInfo(const rclcpp::Time &now_time);
    void printFSMDebugInfo(const rclcpp::Time &now_time);
    void printAprilTagDebugInfo(const rclcpp::Time &now_time);
    void printSE3ControllerDebugInfo(const rclcpp::Time &now_time);
    
    // Public getter methods for external access
    bool rc_is_received(const rclcpp::Time &now_time);
    const State_Data_t& get_state_data() const { return state_data; }
    const TF_Data_t& get_tf_data() const { return tf_data; }

private:
    // Node handle
    Parameter_t &param;

    // FSM state
    FSM_State state;
    FSM_State previous_state;  // Track previous state for transition detection
    Eigen::Vector4d hover_pose;
    bool have_imu;
    
    // Hover target state
    Eigen::Vector3d target_position_;
    double target_yaw_;
    bool target_initialized_;

    // Timestamp manager
    std::unique_ptr<TimestampManager> timestamp_manager;
    rclcpp::Time last_set_offboard_time;
    rclcpp::Time last_set_hover_pose_time;
    rclcpp::Time hover_start_time;  // 悬停开始时间，用于启控延时

    // SE3 Controller
    std::unique_ptr<se3control::SE3Controller> se3_controller;
    
    // 共享的UAVState对象，直接由传感器回调填充
    std::unique_ptr<se3control::UAVState> current_uav_state;

    // Data containers
    State_Data_t state_data;
    LocalPosition_Data_t local_pos_data;
    Imu_Data_t imu_data;
    RC_Data_t rc_data;
    HoverThrustEstimate_Data_t hover_thrust_data;
    TF_Data_t tf_data;

    // CSV logging removed

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_setpoint_pub;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr state_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_vel_sub;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr rc_sub;
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;

    // Service clients
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client;
    
    // Timer for control loop
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Debug print control
    int debug_print_counter_;

    // Initialization methods
    void initializePublishers();
    void initializeSubscribers();

    // Publisher methods
    void publish_thrust_torque_setpoints(const se3control::ControlOutput &out, const rclcpp::Time &stamp);
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    // FSM actions
    void arm();
    void disarm();
    bool toggle_offboard_mode(bool on_off);
    bool toggle_arm_disarm(bool arm);

    // Timeout checks
    bool cmd_is_received(const rclcpp::Time &now_time);
    bool imu_is_received(const rclcpp::Time &now_time);
    bool local_pos_is_received(const rclcpp::Time &now_time);
    bool tf_is_received(const rclcpp::Time &now_time);

    // State machine helper functions
    Desired_State_t get_hover_des();
    se3control::UAVState getCurrentUAVState(double timestamp_seconds);
    se3control::UAVCommand createUAVCommand(const Eigen::Vector3d& position, 
                                           const Eigen::Vector3d& velocity, 
                                           const Eigen::Vector3d& acceleration, 
                                           double yaw);
    
    // Parameter printing for verification
    void printControllerParameters();
    
    // TF callback function
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    
    // AprilTag-based position control
    void calculateTargetPositionFromAprilTag();
    
    // Fixed target position control
    void setFixedTargetPosition();
    
};

#endif