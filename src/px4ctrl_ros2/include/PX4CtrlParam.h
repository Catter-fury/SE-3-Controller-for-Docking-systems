#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <rclcpp/rclcpp.hpp>

class Parameter_t
{
public:
	struct ThrustMapping
	{
		double hover_percentage;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	// Removed AutoTakeoffLand struct as requested

	struct SE3ControllerParams
	{
		// 位置控制增益
		double Kp_x, Kp_y, Kp_z;
		double Kv_x, Kv_y, Kv_z;
		double Kvi_x, Kvi_y, Kvi_z;
		
		// 姿态控制增益
		double Kr_x, Kr_y, Kr_z;
		double Kw_x, Kw_y, Kw_z;
		double Kri_x, Kri_y, Kri_z;
		
		// 物理参数
		double mass;
		double gravity;
		double Ixx, Iyy, Izz;
		
		// 控制限制
		double max_thrust;
		double min_thrust;
		double max_torque;
		
		// 积分控制
		bool use_integral;
	};

	struct ThrustLimits
	{
		// 推力限制
		double max_thrust;  // -0.42
		double min_thrust;  // -0.55
	};

	struct TorqueLimits
	{
		// 力矩限制
		double max_torque;  // 0.3
		double min_torque;  // -0.3
	};

	struct KFnoise
	{
		//KF噪声
        double Q_pos;  // 位置过程噪声
        double Q_vel;  // 速度过程噪声
        double R_pos;   // 位置测量噪声
    };

	ThrustMapping thr_map;
	RCReverse rc_reverse;
	SE3ControllerParams se3;
	ThrustLimits thrust_limits;
	TorqueLimits torque_limits;
	KFnoise kf;

	double ctrl_freq_max;
	double max_manual_vel;

	Parameter_t();
	void config_from_ros_handle(std::shared_ptr<rclcpp::Node> nh);
	void config_full_thrust(double hov);

private:
	template<typename T>
	void read_essential_param(std::shared_ptr<rclcpp::Node> nh, const std::string &name, T &val);
};

#endif