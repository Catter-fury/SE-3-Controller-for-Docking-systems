#include "../include/PX4CtrlParam.h"

// Lightweight macro for parameter loading - minimal intrusion, type safe
#define LOAD_PARAM(node, key, ref) read_essential_param(node, key, ref)

Parameter_t::Parameter_t()
{
	// 推力映射参数
	thr_map.hover_percentage = 0.5;
	
	// RC反向参数
	rc_reverse.roll = false; rc_reverse.pitch = false; 
	rc_reverse.yaw = false; rc_reverse.throttle = false;
	
	// Removed takeoff/land parameters as requested
	
	// SE3控制器参数
	se3.Kp_x = 4.0; se3.Kp_y = 4.0; se3.Kp_z = 5.0;
	se3.Kv_x = 2.8; se3.Kv_y = 2.8; se3.Kv_z = 3.6;
	se3.Kvi_x = 0.5; se3.Kvi_y = 0.5; se3.Kvi_z = 0.5;
	
	se3.Kr_x = 1.2; se3.Kr_y = 1.2; se3.Kr_z = 0.3;
	se3.Kw_x = 0.3; se3.Kw_y = 0.3; se3.Kw_z = 0.05;
	se3.Kri_x = 0.1; se3.Kri_y = 0.1; se3.Kri_z = 0.1;
	
	se3.mass = 1.0; se3.gravity = 9.81;
	se3.Ixx = 0.02; se3.Iyy = 0.02; se3.Izz = 0.04;
	
	se3.max_thrust = -0.1; se3.min_thrust = -0.8;
	se3.max_torque = 0.55;
	
	se3.use_integral = true;
	
	// 推力限制参数
	thrust_limits.max_thrust = -0.42;
	thrust_limits.min_thrust = -0.55;
	
	// 力矩限制参数
	torque_limits.max_torque = 0.3;
	torque_limits.min_torque = -0.3;
	
	// 基础参数
	ctrl_freq_max = 250.0;
	max_manual_vel = 2.0;

	kf.Q_pos = 0.01;  // 位置过程噪声
	kf.Q_vel = 0.01;  // 速度过程噪声
	kf.R_pos = 0.1;   // 位置测量噪声
}

void Parameter_t::config_from_ros_handle(std::shared_ptr<rclcpp::Node> nh)
{
	// 推力映射参数
	LOAD_PARAM(nh, "thr_map.hover_percentage", thr_map.hover_percentage);

	// 基础参数
	LOAD_PARAM(nh, "ctrl_freq_max", ctrl_freq_max);
	LOAD_PARAM(nh, "max_manual_vel", max_manual_vel);

	// RC 反向参数
	LOAD_PARAM(nh, "rc_reverse.roll", rc_reverse.roll);
	LOAD_PARAM(nh, "rc_reverse.pitch", rc_reverse.pitch);
	LOAD_PARAM(nh, "rc_reverse.yaw", rc_reverse.yaw);
	LOAD_PARAM(nh, "rc_reverse.throttle", rc_reverse.throttle);

	// SE3 位置/速度增益
	LOAD_PARAM(nh, "se3.Kp_x", se3.Kp_x);
	LOAD_PARAM(nh, "se3.Kp_y", se3.Kp_y);
	LOAD_PARAM(nh, "se3.Kp_z", se3.Kp_z);
	LOAD_PARAM(nh, "se3.Kv_x", se3.Kv_x);
	LOAD_PARAM(nh, "se3.Kv_y", se3.Kv_y);
	LOAD_PARAM(nh, "se3.Kv_z", se3.Kv_z);
	LOAD_PARAM(nh, "se3.Kvi_x", se3.Kvi_x);
	LOAD_PARAM(nh, "se3.Kvi_y", se3.Kvi_y);
	LOAD_PARAM(nh, "se3.Kvi_z", se3.Kvi_z);

	// SE3 姿态/角速度/积分增益
	LOAD_PARAM(nh, "se3.Kr_x", se3.Kr_x);
	LOAD_PARAM(nh, "se3.Kr_y", se3.Kr_y);
	LOAD_PARAM(nh, "se3.Kr_z", se3.Kr_z);
	LOAD_PARAM(nh, "se3.Kw_x", se3.Kw_x);
	LOAD_PARAM(nh, "se3.Kw_y", se3.Kw_y);
	LOAD_PARAM(nh, "se3.Kw_z", se3.Kw_z);
	LOAD_PARAM(nh, "se3.Kri_x", se3.Kri_x);
	LOAD_PARAM(nh, "se3.Kri_y", se3.Kri_y);
	LOAD_PARAM(nh, "se3.Kri_z", se3.Kri_z);

	// 物理/限制/积分开关
	LOAD_PARAM(nh, "se3.mass", se3.mass);
	LOAD_PARAM(nh, "se3.gravity", se3.gravity);
	LOAD_PARAM(nh, "se3.Ixx", se3.Ixx);
	LOAD_PARAM(nh, "se3.Iyy", se3.Iyy);
	LOAD_PARAM(nh, "se3.Izz", se3.Izz);
	LOAD_PARAM(nh, "se3.max_thrust", se3.max_thrust);
	LOAD_PARAM(nh, "se3.min_thrust", se3.min_thrust);
	LOAD_PARAM(nh, "se3.max_torque", se3.max_torque);
	LOAD_PARAM(nh, "se3.use_integral", se3.use_integral);

	// 推力/力矩限幅
	LOAD_PARAM(nh, "thrust_limits.max_thrust", thrust_limits.max_thrust);
	LOAD_PARAM(nh, "thrust_limits.min_thrust", thrust_limits.min_thrust);
	LOAD_PARAM(nh, "torque_limits.max_torque", torque_limits.max_torque);
	LOAD_PARAM(nh, "torque_limits.min_torque", torque_limits.min_torque);

	// Kalman滤波参数
	LOAD_PARAM(nh, "kf.Q_pos", kf.Q_pos);
	LOAD_PARAM(nh, "kf.Q_vel", kf.Q_vel);
	LOAD_PARAM(nh, "kf.R_pos", kf.R_pos);
	
}

void Parameter_t::config_full_thrust(double hov)
{
	// Implementation for full thrust calculation based on hover percentage
	thr_map.hover_percentage = hov;
}

template<typename T>
void Parameter_t::read_essential_param(std::shared_ptr<rclcpp::Node> nh, const std::string &name, T &val)
{
	// Declare parameter if it doesn't exist, then get its value
	if (!nh->has_parameter(name)) {
		nh->declare_parameter(name, val);
	}
	
	try {
		val = nh->get_parameter(name).get_value<T>();
		RCLCPP_DEBUG(nh->get_logger(), "Successfully loaded parameter '%s'", name.c_str());
	} catch (const std::exception& e) {
		RCLCPP_WARN(nh->get_logger(), "Failed to load parameter '%s': %s. Using default value.", name.c_str(), e.what());
	}
}

