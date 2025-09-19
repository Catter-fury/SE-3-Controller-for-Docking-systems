#ifndef SE3CON_HPP
#define SE3CON_HPP

#include "KF_filter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include "input.h"

// Forward declaration instead of include to avoid circular dependency
class Parameter_t;

// 类型定义，适配项目 - 使用更具体的名称避免与Eigen冲突
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Quaterniond = Eigen::Quaterniond;


namespace se3control {

// 无人机状态结构
struct UAVState {
    Vector3d position;           // 位置 [x, y, z]
    Vector3d velocity;           // 速度 [vx, vy, vz]
    Matrix3d rotation;           // 旋转矩阵 R
    Vector3d angular_velocity;   // 角速度 [wx, wy, wz]
    double hover_thrust;        // 悬停推力值
    double timestamp;           // 时间戳
    
    UAVState() : 
        position(Vector3d::Zero()),
        velocity(Vector3d::Zero()),
        rotation(Matrix3d::Identity()),
        angular_velocity(Vector3d::Zero()),
        hover_thrust(-0.5),
        timestamp(0.0) {}
};

// 无人机指令结构
struct UAVCommand {
    Vector3d position;           // 期望位置
    Vector3d velocity;           // 期望速度
    Vector3d acceleration;       // 期望加速度
    Vector3d b1d;               // 期望方向向量
    double yaw_desired;         // 期望偏航角
    Matrix3d Rd;                // 期望旋转矩阵
    Matrix3d Rd_dot;            // 期望旋转矩阵一阶导数
    Matrix3d Rd_ddot;           // 期望旋转矩阵二阶导数
    Vector3d Wd;                // 期望角速度
    Vector3d Wd_dot;            // 期望角加速度
    
    UAVCommand() :
        position(Vector3d::Zero()),
        velocity(Vector3d::Zero()),
        acceleration(Vector3d::Zero()),
        b1d(Vector3d::UnitX()),
        yaw_desired(0.0),
        Rd(Matrix3d::Identity()),
        Rd_dot(Matrix3d::Zero()),
        Rd_ddot(Matrix3d::Zero()),
        Wd(Vector3d::Zero()),
        Wd_dot(Vector3d::Zero()) {}
};

// 控制器参数配置
struct ControllerGains {
    Vector3d position_gains;            // 位置控制增益 [Kp_x, Kp_y, Kp_z]
    Vector3d velocity_gains;            // 速度控制增益 [Kv_x, Kv_y, Kv_z]
    Vector3d attitude_gains;            // 姿态控制增益 [Kr_x, Kr_y, Kr_z]
    Vector3d angular_velocity_gains;    // 角速度控制增益 [Kw_x, Kw_y, Kw_z]
    Vector3d integral_position_gains;   // 位置积分增益 [Kvi_x, Kvi_y, Kvi_z]
    Vector3d integral_attitude_gains;   // 姿态积分增益 [Kri_x, Kri_y, Kri_z]
    bool use_integral;                 // 是否使用积分控制
    
    ControllerGains() :
        position_gains(Vector3d::Zero()),
        velocity_gains(Vector3d::Zero()),
        attitude_gains(Vector3d::Zero()),
        angular_velocity_gains(Vector3d::Zero()),
        integral_position_gains(Vector3d::Zero()),
        integral_attitude_gains(Vector3d::Zero()),
        use_integral(false) {}
};

struct UAVParameters {
    double mass;                // 质量 [kg]
    double gravity;             // 重力加速度 [m/s^2]
    Matrix3d inertia;           // 惯性矩阵 [kg*m^2]
    double max_thrust;         // 最大推力
    double min_thrust;         // 最小推力
    double max_torque;         // 最大力矩
    
    UAVParameters() :
        mass(2.0),
        gravity(9.81),
        inertia(Matrix3d::Identity()),
        max_thrust(1.0),
        min_thrust(0.0),
        max_torque(1.0) {}
};

struct ControllerConfig {
    ControllerGains gains;
    UAVParameters uav_params;
    double dt;                 // 时间步长
    
    ControllerConfig() : dt(0.01) {}
};

// 控制输出结构
struct ControlOutput {
    double thrust;              // 推力 (标量，兼容旧接口)
    Vector3d thrust_vector;      // 完整推力向量 [Fx, Fy, Fz] (牛顿)
    Vector3d moment;            // 力矩 [Mx, My, Mz]
    Vector3d A;                 // 期望加速度向量
    Vector3d eR;                // 旋转误差
    Vector3d eW;                // 角速度误差
    Matrix3d Rd;                // 期望旋转矩阵
    Matrix3d Rd_dot;            // 期望旋转矩阵导数
    Matrix3d Rd_ddot;           // 期望旋转矩阵二阶导数
    Vector3d Wd;                // 期望角速度
    Vector3d Wd_dot;            // 期望角加速度
    Vector3d b1c, b2c, b3c;     // 期望坐标系基向量
    Matrix3d RdtR;              // Rd^T * R
    
    ControlOutput() :
        thrust(0.0),
        moment(Vector3d::Zero()),
        A(Vector3d::Zero()),
        eR(Vector3d::Zero()),
        eW(Vector3d::Zero()),
        Rd(Matrix3d::Identity()),
        Rd_dot(Matrix3d::Zero()),
        Rd_ddot(Matrix3d::Zero()),
        Wd(Vector3d::Zero()),
        Wd_dot(Vector3d::Zero()),
        b1c(Vector3d::UnitX()),
        b2c(Vector3d::UnitY()),
        b3c(Vector3d::UnitZ()),
        RdtR(Matrix3d::Identity()) {}
};


// SE(3) 反对称矩阵算子 (hat map)
inline Matrix3d hat(const Vector3d& v) {
    Matrix3d result;
    result << 0.0, -v(2), v(1),
              v(2), 0.0, -v(0),
              -v(1), v(0), 0.0;
    return result;
}

// SE(3) vee算子 (从反对称矩阵提取向量)
inline Vector3d vee(const Matrix3d& M) {
    return Vector3d(M(2,1), M(0,2), M(1,0));
}

// SE3控制器类
class SE3Controller {
public:
    SE3Controller();
    ~SE3Controller() = default;
    
    // 初始化函数
    bool initialize(const Parameter_t& param);
    bool initialize(const ControllerConfig& config);
    bool initializeWithROS2Params(rclcpp::Node::SharedPtr node);
    
    // 主要控制函数
    ControlOutput computeControl(const UAVState& current_state, const Desired_State_t& des);
    ControlOutput computeControl(const UAVState& current_state, UAVCommand& desired_command);
    
    // 重置函数
    
    // 时间管理
    void updateTime(double time);
    void setTimeDelta(double dt);
    double getTimeDelta() const;
    
    // convertToUAVState函数已删除
    
    // Legacy function for test node compatibility 
    void convertFromControlOutput(const ControlOutput& output, Controller_Output_t& u);
    
    // Logging interface - allows injection of ROS2 node for unified logging
    void set_logger_node(rclcpp::Node* node);

    // 使用KF滤波器作为输入
    ControlOutput computeControl(const UAVState& current_state, const KF_Filter& filter,UAVCommand& desired_command);
                                           
private:
    // 内部成员变量
    bool initialized_;
    ControllerConfig config_;
    
    // 积分误差
    Vector3d position_integral_error_;
    Vector3d attitude_integral_error_;
    double attitude_integral_error_roll_;
    double attitude_integral_error_pitch_;
    double attitude_integral_error_yaw_;
    
    // 时间管理
    double current_time_;
    double previous_time_;
    
    // 历史状态用于导数计算
    bool first_iteration_;
    Matrix3d Rd_prev_;
    Matrix3d Rd_dot_prev_;
    Vector3d Wd_prev_;
    
    // 静态基向量
    static const Vector3d e1_;
    static const Vector3d e2_;
    static const Vector3d e3_;
    
    // Optional logger node for unified logging (nullptr means use std::cerr)
    rclcpp::Node* logger_node_;
    
    // 内部控制函数
    void geometricPositionControl(const UAVState& state, UAVCommand& command,
                                 double& f_total, Vector3d& A);
    
    void geometricAttitudeControl(const UAVState& state, UAVCommand& command,
                                 Vector3d& M, const Vector3d& A, Vector3d& eR, Vector3d& eW,
                                 Matrix3d& RdtR, Vector3d& b1c, Vector3d& b2c, Vector3d& b3c);
    
    void computeDesiredRotationAndDerivatives(const Vector3d& A, UAVCommand& command,
                                            Vector3d& b1c, Vector3d& b2c, Vector3d& b3c);
    
    // 辅助函数
    Matrix3d computeRotationError(const Matrix3d& current_rotation,
                                const Matrix3d& desired_rotation);
    
    Vector3d matToSo3Lie(const Matrix3d& R);
    Vector3d handlePiRotation(const Matrix3d& R);
    
    Vector3d saturate(const Vector3d& input, double max_value);
    void saturate(double& value, double min_val, double max_val);
    
    void applyControlLimits(ControlOutput& output);
    
    // 验证函数
    bool validateConfiguration() const;
    bool isValidState(const UAVState& state) const;
    bool isValidCommand(const UAVCommand& command) const;
    bool isValidOutput(const ControlOutput& output) const;
    
    // 统一初始化逻辑 - consolidates parameter loading and validation
    bool loadFromParam(const Parameter_t& param, ControllerConfig& config);
};

} // namespace se3control

#endif // SE3CON_HPP