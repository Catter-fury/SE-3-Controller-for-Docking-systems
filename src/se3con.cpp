#include "se3_control_pkg/se3con.hpp"
#include"se3_control_pkg/KF_filter.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>
#include <Eigen/Dense>


// 编译器优化提示 预处理器宏定义，主要目的是为了进行性能优化和保持跨编译器兼容性
#ifdef __GNUC__
    #define LIKELY(x)   __builtin_expect(!!(x), 1)
    #define UNLIKELY(x) __builtin_expect(!!(x), 0)
    #define FORCE_INLINE __attribute__((always_inline))
#else
    #define LIKELY(x)   (x)
    #define UNLIKELY(x) (x)
    #define FORCE_INLINE inline
#endif

namespace se3control {

const Vector3d SE3Controller::e1_ = Vector3d::UnitX();
const Vector3d SE3Controller::e2_ = Vector3d::UnitY();
const Vector3d SE3Controller::e3_ = Vector3d::UnitZ();

// 构造函数
SE3Controller::SE3Controller() : 
    initialized_(false),
    position_integral_error_(Vector3d::Zero()),
    attitude_integral_error_(Vector3d::Zero()),
    attitude_integral_error_roll_(0.0),
    attitude_integral_error_pitch_(0.0),
    attitude_integral_error_yaw_(0.0),
    current_time_(0.0),
    previous_time_(0.0),
    first_iteration_(true),
    Rd_prev_(Matrix3d::Identity()),
    Rd_dot_prev_(Matrix3d::Zero()),
    Wd_prev_(Vector3d::Zero()) {
}


// 使用ROS2参数服务器初始化控制器
bool SE3Controller::initializeWithROS2Params(rclcpp::Node::SharedPtr node) {
    // 创建Parameter_t并加载配置
    Parameter_t param;
    param.config_from_ros_handle(node);
    
    // 使用Parameter_t初始化
    return initialize(param);
}

// 使用Parameter_t参数初始化控制器
bool SE3Controller::initialize(const Parameter_t& param) {
    // 从Parameter_t.se3转换为ControllerConfig
    config_.uav_params.mass = param.se3.mass;
    config_.uav_params.gravity = param.se3.gravity;
    
    // 设置惯性矩阵为对角矩阵
    config_.uav_params.inertia = Matrix3d::Identity();
    config_.uav_params.inertia(0,0) = param.se3.Ixx;  // Ixx
    config_.uav_params.inertia(1,1) = param.se3.Iyy;  // Iyy  
    config_.uav_params.inertia(2,2) = param.se3.Izz;  // Izz
    
    // 设置控制增益
    config_.gains.position_gains = Vector3d(param.se3.Kp_x, param.se3.Kp_y, param.se3.Kp_z);
    config_.gains.velocity_gains = Vector3d(param.se3.Kv_x, param.se3.Kv_y, param.se3.Kv_z);
    config_.gains.attitude_gains = Vector3d(param.se3.Kr_x, param.se3.Kr_y, param.se3.Kr_z);
    config_.gains.angular_velocity_gains = Vector3d(param.se3.Kw_x, param.se3.Kw_y, param.se3.Kw_z);
    
    // 积分增益
    config_.gains.integral_position_gains = Vector3d(param.se3.Kvi_x, param.se3.Kvi_y, param.se3.Kvi_z);
    config_.gains.integral_attitude_gains = Vector3d(param.se3.Kri_x, param.se3.Kri_y, param.se3.Kri_z);
    config_.gains.use_integral = param.se3.use_integral;
    
    // 控制限制
    config_.uav_params.max_thrust = param.se3.max_thrust;
    config_.uav_params.min_thrust = param.se3.min_thrust;
    config_.uav_params.max_torque = param.se3.max_torque;
    
    // 时间步长
    config_.dt = 1.0 / param.ctrl_freq_max;
    
    // 验证配置
    if (!validateConfiguration()) {
        std::cerr << "Invalid configuration detected" << std::endl;
        return false;
    }
    
    initialized_ = true;
    
    return true;
}

// 使用预定义配置结构体初始化控制器
bool SE3Controller::initialize(const ControllerConfig& config) {
    // 验证配置的合理性
    config_ = config;
    if (!validateConfiguration()) {
        std::cerr << "Invalid configuration provided" << std::endl;
        return false;
    }
    
    initialized_ = true;
    
    return true;
}

// 计算控制输出（支持Desired_State_t）
ControlOutput SE3Controller::computeControl(const UAVState& current_state, 
                                           const Desired_State_t& des) {
    // 将Desired_State_t转换为UAVCommand
    UAVCommand desired_command;
    desired_command.position = des.p;
    desired_command.velocity = des.v;
    desired_command.acceleration = des.a;
    // 从期望偏航角直接计算b1d向量
    desired_command.b1d = Vector3d(cos(des.yaw), sin(des.yaw), 0.0);
    desired_command.yaw_desired = des.yaw;  // 保留偏航角信息
        
    return computeControl(current_state, desired_command);
}

// 主要的控制函数实现
ControlOutput SE3Controller::computeControl(const UAVState& current_state, UAVCommand& desired_command) {
    updateTime(current_state.timestamp);

    // 检查控制器是否已初始化
    if (UNLIKELY(!initialized_)) {
        std::cerr << "SE3Controller: Controller not initialized!" << std::endl;
        return ControlOutput{};
    }
    
    // 验证输入状态的有效性
    if (UNLIKELY(!isValidState(current_state))) {
        std::cerr << "SE3Controller: Invalid state input detected!" << std::endl;
        return ControlOutput{};
    }
    
    // 验证输入指令的有效性
    if (UNLIKELY(!isValidCommand(desired_command))) {
        std::cerr << "SE3Controller: Invalid command input detected!" << std::endl;
        return ControlOutput{};
    }

    ControlOutput output;
    double f_total = 0.0;
    Vector3d M = Vector3d::Zero();
    Vector3d eR, eW;
    Matrix3d RdtR;
    Vector3d b1c, b2c, b3c;
    
    try {
        // 几何位置控制 - 计算期望旋转矩阵和推力
        Vector3d A; 
        geometricPositionControl(current_state, desired_command, f_total, A);
        
        // 姿态控制
        geometricAttitudeControl(current_state, desired_command, M, A, eR, eW, RdtR, b1c, b2c, b3c);
        
        output.thrust = f_total;
        output.thrust_vector = A;
        output.moment = M;
        output.A = A;
        output.eR = eR;
        output.eW = eW;
        output.Rd = desired_command.Rd;
        output.Rd_dot = desired_command.Rd_dot;
        output.Rd_ddot = desired_command.Rd_ddot;
        output.Wd = desired_command.Wd;
        output.Wd_dot = desired_command.Wd_dot;
        output.b1c = b1c;
        output.b2c = b2c;
        output.b3c = b3c;
        output.RdtR = RdtR;

        applyControlLimits(output);
        
    } catch (const std::exception& e) {
        std::cerr << "SE3Controller: Exception in control computation: " << e.what() << std::endl;
        return ControlOutput{};
    }
    
    return output;
}

//获取KF的状态数据 - 已删除，因为KF_filter接口不匹配

// 计算控制输出（主要函数）
ControlOutput SE3Controller::computeControl(const UAVState& current_state, const KF_Filter& filter,
                                           UAVCommand& desired_command) {


     // 从KF滤波器获取UAV状态
    UAVState filtered_state = filter.getUAVState();
    
    // 保留原始状态中的姿态信息（滤波器只提供位置和速度）
    filtered_state.rotation = current_state.rotation;
    filtered_state.angular_velocity = current_state.angular_velocity;
    filtered_state.hover_thrust = current_state.hover_thrust;
    
    // 使用滤波后的状态更新时间
    updateTime(filtered_state.timestamp); 

    // 检查控制器是否已初始化
    if (UNLIKELY(!initialized_)) {
        std::cerr << "SE3Controller: Controller not initialized!" << std::endl;
        return ControlOutput{};
    }
    
    // 验证输入状态的有效性 - 增强错误处理
    if (UNLIKELY(!isValidState(filtered_state))) {
        std::cerr << "SE3Controller: Invalid state input detected - ";
        if (!filtered_state.position.allFinite()) std::cerr << "position ";
        if (!filtered_state.velocity.allFinite()) std::cerr << "velocity ";
        if (!filtered_state.rotation.allFinite()) std::cerr << "rotation ";
        if (!filtered_state.angular_velocity.allFinite()) std::cerr << "angular_velocity ";
        std::cerr << std::endl;
        return ControlOutput{};
    }
    
    // 验证输入指令的有效性
    if (UNLIKELY(!isValidCommand(desired_command))) {
        std::cerr << "SE3Controller: Invalid command input detected!" << std::endl;
        return ControlOutput{};
    }
    
    // 输出变量
    ControlOutput output;
    double f_total = 0.0;  
    Vector3d M = Vector3d::Zero();
    Vector3d eR, eW;
    Matrix3d RdtR;
    Vector3d b1c, b2c, b3c;
    
    try {
        // 几何位置控制 - 计算期望旋转矩阵和推力
        Vector3d A; 
        geometricPositionControl(filtered_state, desired_command, f_total,A);
        
        // 姿态控制
        geometricAttitudeControl(filtered_state, desired_command, M, A, eR, eW, RdtR, b1c, b2c, b3c);
        
        output.thrust = f_total;
        output.thrust_vector = A;  // 输出完整的3D推力向量
        output.moment = M;
        output.A = A;
        output.eR = eR;
        output.eW = eW;
        output.Rd = desired_command.Rd;
        output.Rd_dot = desired_command.Rd_dot;
        output.Rd_ddot = desired_command.Rd_ddot;
        output.Wd = desired_command.Wd;
        output.Wd_dot = desired_command.Wd_dot;
        output.b1c = b1c;
        output.b2c = b2c;
        output.b3c = b3c;
        output.RdtR = RdtR;
        
        // 应用控制输出限幅
        applyControlLimits(output);
        
    } catch (const std::exception& e) {
        std::cerr << "SE3Controller: Exception in control computation: " << e.what() << std::endl;
        return ControlOutput{};
    }
    
    return output;
}

// 几何位置控制器 - 基于SE(3)理论
void SE3Controller::geometricPositionControl(const UAVState& state, UAVCommand& command,
                                            double& f_total,Vector3d& A) {
    // 平移误差函数 - 方程 (11), (12)
    Vector3d eX = state.position - command.position;    // 位置误差
    Vector3d eV = state.velocity - command.velocity;    // 速度误差

    // 位置积分项 - 优化：避免创建临时对象
    if (LIKELY(config_.gains.use_integral)) {
        // 直接更新积分误差，避免创建临时Vector3d对象
        position_integral_error_ += eX * config_.dt;
        
        constexpr double sat_sigma = 1.8;  // 使用constexpr提高编译时优化
        position_integral_error_ = saturate(position_integral_error_, sat_sigma);
    } else {
        position_integral_error_.setZero();
    }
    
    // 沿负b3轴的力 'f' - 方程 (14) - 计算当前重力
    const double mg = config_.uav_params.mass * config_.uav_params.gravity;
    
    
    // 避免多次矩阵乘法，直接计算 (添加积分项)
    A.noalias() = -config_.gains.position_gains.cwiseProduct(eX)     
                  - config_.gains.velocity_gains.cwiseProduct(eV)   
                  - config_.gains.integral_position_gains.cwiseProduct(position_integral_error_)  // 添加位置积分项
                  - mg * e3_  
                  + config_.uav_params.mass * command.acceleration;
    
    // 直接使用旋转矩阵第三列，避免矩阵乘法
    const Vector3d& b3 = state.rotation.col(2);  
    f_total = A.dot(b3);
    
    // 将物理推力转换为油门值 - 优化：使用预计算值，减少除法和分支
    const double inv_hover_force = 1.0 / mg;  // 预计算倒数，避免除法操作
    const double abs_hover_thrust = std::abs(state.hover_thrust);
    
    // 优化的转换逻辑：使用数学函数代替分支
    /*
    
    */
    f_total = (f_total * inv_hover_force) * abs_hover_thrust;
    //f_total = -0.80515;
    
    
    f_total = std::copysign(f_total, -1.0);  // 始终保持负值（向上推力）
    

    
}

// 几何姿态控制器 - 标准SE(3)控制
void SE3Controller::geometricAttitudeControl(const UAVState& state, UAVCommand& command, 
                                             Vector3d& M, const Vector3d& A, Vector3d& eR, Vector3d& eW, Matrix3d& RdtR, Vector3d& b1c, Vector3d& b2c, Vector3d& b3c) {
    // 标准SE(3)论文中的几何控制器
    //计算期望角速度与角加速度
    computeDesiredRotationAndDerivatives(A,command,b1c,b2c,b3c);
    RdtR = command.Rd.transpose() * state.rotation;
    eR = 0.5 * vee(RdtR - RdtR.transpose());  // 旋转误差
    eW = state.angular_velocity - state.rotation.transpose() * command.Rd * command.Wd;  // 角速度误差
    
    // 姿态积分项 - 优化：避免临时对象
    if (LIKELY(config_.gains.use_integral)) {
        attitude_integral_error_ += eR * config_.dt;
        // 姿态积分限幅
        constexpr double attitude_sat = 0.5;
        attitude_integral_error_ = saturate(attitude_integral_error_, attitude_sat);
    } else {
        attitude_integral_error_.setZero();
    }
    
    // 计算控制力矩 - 优化：减少重复计算
    static thread_local Vector3d RTRd_Wd;  // 缓存重复计算结果
    RTRd_Wd.noalias() = state.rotation.transpose() * command.Rd * command.Wd;
    
    M.noalias() = -config_.gains.attitude_gains.cwiseProduct(eR)  
                  - config_.gains.angular_velocity_gains.cwiseProduct(eW)  
                  - config_.gains.integral_attitude_gains.cwiseProduct(attitude_integral_error_)  // 添加姿态积分项
                  + hat(RTRd_Wd) * config_.uav_params.inertia * RTRd_Wd
                  + config_.uav_params.inertia * state.rotation.transpose() * command.Rd * command.Wd_dot;


}


// 计算期望旋转矩阵及其导数
void SE3Controller::computeDesiredRotationAndDerivatives(const Vector3d& A, 
                                                         UAVCommand& command, Vector3d& b1c, Vector3d& b2c, Vector3d& b3c) {
    // 计算期望的机体z轴方向(b3c)
    // A向量指向期望加速度方向，但推力应该与加速度反向
    b3c = (-A).normalized();

    //计算b2c
    b2c = b3c.cross(command.b1d).normalized();
    //b2c = (hat(b3c)*command.b1d).normalized();

    //计算b1c
    b1c = b2c.cross(b3c);
    //b1c = hat(b2c)*b3c;

    // 构建期望旋转矩阵
    
    //command.Rd = Matrix3d::Identity(); //期望旋转矩阵设置为单位矩阵
    command.Rd.col(0) = b1c;  // x轴
    command.Rd.col(1) = b2c;  // y轴
    command.Rd.col(2) = b3c;  // z轴
                                                     
    /*
    command.Rd.col(0) = b1c;  // x轴
    command.Rd.col(1) = b2c;  // y轴
    command.Rd.col(2) = b3c;  // z轴
    */

    // ========================================================================
    // 改进的期望角速度与期望角加速度计算
    // 基于伪代码算法：更稳定的数值实现
    // ========================================================================
    
    // 分支预测：大多数情况下不是第一次迭代
    if(LIKELY(!first_iteration_ && config_.dt > 1e-8)){
        // ====================================================================
        // 部分1: 计算期望角速度 (Omega_c)
        // ====================================================================
        
        // 步骤1: 计算从上一姿态到当前姿态的旋转增量矩阵
        // 修正计算顺序：delta_R = Rd_prev^T * Rd_current
        Matrix3d delta_R = Rd_prev_.transpose() * command.Rd;
        
        // 步骤2: 改进的对数映射转换为轴角向量
        Vector3d axis_angle_vector = matToSo3Lie(delta_R);
            
        // 步骤3: 计算角速度
        command.Wd = axis_angle_vector / config_.dt;
        
        // ====================================================================
        // 部分2: 计算期望角加速度 (Omega_dot_c) - 改进方法
        // ====================================================================
        
        // 步骤1: 直接从角速度计算当前姿态导数（更稳定）
        command.Rd_dot = command.Rd * hat(command.Wd);
        
        // 步骤2: 计算姿态二阶导数（使用改进的差分）
        Matrix3d Rd_ddot;
        if (Rd_dot_prev_.isZero()) {
            // 第一次有效计算，无法计算二阶导
            Rd_ddot = Matrix3d::Zero();
        } else {
            // 使用后向差分计算二阶导数，添加数值稳定性检查
            if (LIKELY(config_.dt > 1e-8)) {
                Rd_ddot = (command.Rd_dot - Rd_dot_prev_) / config_.dt;
            } else {
                Rd_ddot = Matrix3d::Zero();
            }
        }
        
        // 步骤3: 改进的角加速度计算
        Matrix3d omega_skew = hat(command.Wd);
        Vector3d Wd_dot_raw = vee(command.Rd.transpose() * Rd_ddot - omega_skew * omega_skew);
        
        // 智能限幅：根据当前角速度自适应调整限制，提升数值稳定性
        const double base_limit = 5.0;
        const double velocity_factor = 2.0;
        double adaptive_limit = std::max(base_limit, velocity_factor * command.Wd.norm());
        
        // 添加额外的数值稳定性检查
        if (UNLIKELY(!Wd_dot_raw.allFinite())) {
            command.Wd_dot = Vector3d::Zero();
        } else {
            command.Wd_dot = saturate(Wd_dot_raw, adaptive_limit);
        }
        
    } else {
        // 第一次迭代或时间步长过小的情况
        command.Wd = Vector3d::Zero();
        command.Wd_dot = Vector3d::Zero();
        command.Rd_dot = Matrix3d::Zero();
    }
 
    //更新历史状态
    Rd_prev_ = command.Rd;
    Rd_dot_prev_ = command.Rd_dot;
    Wd_prev_ = command.Wd;   

}




/*其他相关的函数*/

// 计算旋转误差矩阵
Matrix3d SE3Controller::computeRotationError(const Matrix3d& current_rotation,
                                           const Matrix3d& desired_rotation) {
    return 0.5 * (desired_rotation.transpose() * current_rotation - 
                  current_rotation.transpose() * desired_rotation);
}

// hat和vee算子已移至头文件作为内联函数优化

/**
 * 改进的旋转矩阵对数映射实现
 * 对应函数: logmToVector(rotation_matrix_R)
 */
Vector3d SE3Controller::matToSo3Lie(const Matrix3d& R) {
    const double numerical_eps = 1e-8; // 改进的数值精度阈值
    
    // 快速路径：检测近似单位矩阵（极小旋转）
    double trace_R = R.trace();
    double trace_error = std::abs(trace_R - 3.0);
    
    if (LIKELY(trace_error < 1e-6)) {
        // 极小旋转：直接使用线性近似，避免三角函数计算
        // 提升数值稳定性：使用Rodrigues公式的线性近似
        constexpr double half = 0.5;
        return half * Vector3d(R(2,1) - R(1,2), 
                             R(0,2) - R(2,0), 
                             R(1,0) - R(0,1));
    }
    
    // 步骤1: 计算旋转角theta（非快速路径）
    // 提升数值稳定性：更精确的角度计算
    double cos_theta = (trace_R - 1.0) * 0.5;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));  // 限制在[-1,1]范围内
    
    // 使用更稳定的角度计算，避免acos在边界处的数值问题
    double theta;
    if (cos_theta > 0.999999) {
        // 接近0度旋转，使用小角度近似
        theta = std::sqrt(2.0 * (1.0 - cos_theta));
    } else if (cos_theta < -0.999999) {
        // 接近180度旋转
        theta = M_PI;
    } else {
        theta = std::acos(cos_theta);
    }
    
    // 步骤2: 根据旋转角大小分情况处理
    double sin_theta = std::sin(theta);
    
    if (std::abs(sin_theta) < numerical_eps) {
        // 特殊情况处理
        if (theta < numerical_eps) {
            // 情况1: 几乎无旋转 (theta ≈ 0)
            // 使用一阶泰勒展开: log(R) ≈ (R - R^T) / 2
            // 优化：直接计算vee结果，避免中间矩阵
            constexpr double half = 0.5;
            return half * Vector3d(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
        } else {
            // 情况2: 180度旋转 (theta ≈ π)
            // 使用特殊处理避免奇异性
            return handlePiRotation(R);
        }
    } else {
        // 情况3: 一般情况 (0 < theta < π, sin(theta) ≠ 0)
        // 对应公式: [u]_x = (θ / (2*sin(θ))) * (R - R^T)
        // 优化：直接计算结果，避免中间矩阵
        double factor = theta / (2.0 * sin_theta);
        return factor * Vector3d(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    }
}

/**
 * 处理180度旋转的特殊情况 (theta ≈ π)
 * 避免sin(theta) ≈ 0的数值奇异性
 */
Vector3d SE3Controller::handlePiRotation(const Matrix3d& R) {
    const double pi = M_PI;
    const double numerical_eps = 1e-8;
    
    // 方法1: 寻找旋转轴（R+I的零空间）
    // 对于180度旋转，有 R*v = -v，即 (R+I)*v = 0
    Matrix3d M = R + Matrix3d::Identity();
    
    // 优化：使用单次遍历找到最大对角元素
    Vector3d axis;
    double max_diagonal = M(0,0);
    int max_index = 0;
    
    if (M(1,1) > max_diagonal) {
        max_diagonal = M(1,1);
        max_index = 1;
    }
    if (M(2,2) > max_diagonal) {
        max_diagonal = M(2,2);
        max_index = 2;
    }
    
    if (max_diagonal > numerical_eps) {
        axis = M.col(max_index);
    } else {
        // 退化情况，返回零向量
        return Vector3d::Zero();
    }
    
    // 归一化并乘以π
    axis.normalize();
    
    // 确定旋转方向（保证连续性） - 优化：避免临时矩阵对象
    // 直接计算反对称部分的vee结果，避免中间矩阵
    Vector3d skew_vec(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    skew_vec *= 0.5;
    if (skew_vec.dot(axis) < 0) {
        axis = -axis;
    }
    
    return pi * axis;
}

// 向量饱和限制函数 - 优化：避免循环和临时对象
Vector3d SE3Controller::saturate(const Vector3d& input, double max_value) {
    return input.cwiseMax(-max_value).cwiseMin(max_value);
}

// 标量饱和限制函数 - 优化：使用标准库函数
void SE3Controller::saturate(double& value, double min_val, double max_val) {
    value = std::max(min_val, std::min(max_val, value));
}



// 更新时间步长
void SE3Controller::updateTime(double time) {
    previous_time_ = current_time_;
    current_time_ = time;
    
    // 更新配置中的dt用于积分计算
    if (previous_time_ > 0.0) {
        config_.dt = current_time_ - previous_time_;
        // 将dt限制在合理范围内，提升数值稳定性
        constexpr double min_dt = 1e-6;
        constexpr double max_dt = 0.1;
        config_.dt = std::max(min_dt, std::min(max_dt, config_.dt));
    }
}

// 设置时间步长
void SE3Controller::setTimeDelta(double dt) {
    constexpr double min_dt = 1e-6;
    constexpr double max_dt = 0.1;
    config_.dt = std::max(min_dt, std::min(max_dt, dt));
}

// 获取时间步长
double SE3Controller::getTimeDelta() const {
    return config_.dt;
}

// 控制输出限幅函数
void SE3Controller::applyControlLimits(ControlOutput& output) {
    // 推力限制：使用配置参数
    const double min_thrust = config_.uav_params.min_thrust;
    const double max_thrust = config_.uav_params.max_thrust;
    saturate(output.thrust, min_thrust, max_thrust);
    
    // 推力向量限制：防止水平推力过大导致失控
    constexpr double max_horizontal_thrust = 2.0;  // 水平推力限制 (牛顿)
    constexpr double max_vertical_thrust = 20.0;   // 垂直推力限制 (牛顿)
    
    // 限制水平分量
    output.thrust_vector(0) = std::max(-max_horizontal_thrust, std::min(max_horizontal_thrust, output.thrust_vector(0)));
    output.thrust_vector(1) = std::max(-max_horizontal_thrust, std::min(max_horizontal_thrust, output.thrust_vector(1)));
    // 限制垂直分量 (向下为负)
    output.thrust_vector(2) = std::max(-max_vertical_thrust, std::min(-1.0, output.thrust_vector(2)));
    
    // 力矩限制：使用配置参数
    const double moment_limit = config_.uav_params.max_torque;
    output.moment = saturate(output.moment, moment_limit);
    
    // 额外的数值安全检查
    if (UNLIKELY(!std::isfinite(output.thrust))) {
        output.thrust = (min_thrust + max_thrust) * 0.5;  // 使用中值作为安全默认值
    }
    if (UNLIKELY(!output.moment.allFinite())) {
        output.moment.setZero();
    }
}

// 验证配置参数的合理性
bool SE3Controller::validateConfiguration() const {
    // 检查基本参数
    if (config_.uav_params.mass <= 0.0) {
        std::cerr << "Invalid mass: " << config_.uav_params.mass << std::endl;
        return false;
    }
    
    if (config_.uav_params.gravity <= 0.0) {
        std::cerr << "Invalid gravity: " << config_.uav_params.gravity << std::endl;
        return false;
    }
    
    if (config_.dt <= 0.0 || config_.dt > 1.0) {
        std::cerr << "Invalid dt: " << config_.dt << std::endl;
        return false;
    }
    
    // 检查增益矩阵的对角元素为正
    if ((config_.uav_params.inertia.diagonal().array() <= 0.0).any()) {
        std::cerr << "Invalid inertia matrix diagonal" << std::endl;
        return false;
    }
    
    // 检查控制增益为正
    if ((config_.gains.position_gains.array() < 0.0).any() ||
        (config_.gains.velocity_gains.array() < 0.0).any() ||
        (config_.gains.attitude_gains.array() < 0.0).any() ||
        (config_.gains.angular_velocity_gains.array() < 0.0).any()) {
        std::cerr << "Invalid control gains (must be non-negative)" << std::endl;
        return false;
    }
    
    return true;
}

// 验证状态输入的有效性
bool SE3Controller::isValidState(const UAVState& state) const {
    return state.position.allFinite() && 
           state.velocity.allFinite() && 
           state.rotation.allFinite() && 
           state.angular_velocity.allFinite() &&
           std::abs(state.rotation.determinant() - 1.0) < 1e-3;  // 检查旋转矩阵的有效性
}

// 验证指令输入的有效性
bool SE3Controller::isValidCommand(const UAVCommand& command) const {
    return command.position.allFinite() && 
           command.velocity.allFinite() && 
           command.acceleration.allFinite() &&
           command.b1d.allFinite() &&
           command.b1d.norm() > 1e-6;  // 检查期望方向向量非零
}

// 验证输出的有效性
bool SE3Controller::isValidOutput(const ControlOutput& output) const {
    return std::isfinite(output.thrust) && 
           output.moment.allFinite() &&
           output.A.allFinite() &&
           output.eR.allFinite() &&
           output.eW.allFinite();
}

// convertToUAVState函数已删除，因为数据结构简化后不再需要

// Legacy function for test node compatibility
void SE3Controller::convertFromControlOutput(const ControlOutput& output, Controller_Output_t& u) {
    // 推力（保持负值，表示向上推力）
    u.thrust = Vector3d(0.0, 0.0, output.thrust);
    
    // 力矩
    u.torque = output.moment;
}

} // namespace se3control