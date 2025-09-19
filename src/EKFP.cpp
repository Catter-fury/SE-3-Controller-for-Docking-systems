#include "se3_control_pkg/se3con.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>
#include <Eigen/Dense>

//EKF预测函数
void SE3ControlNode::Prediction(MatrixXd& P, MatrixXd& J1,const VectorXd& x,const MatrixXd& Q,const VectorXd& uState, int n) {
    // 保存上一次的雅可比矩阵
    MatrixXd J0 = J1;
    
    // 计算新的雅可比矩阵
    J1 = JacobianMatrixQuadrotor6DOF(x, uState);
    
    // 计算先验状态估计值
    xnew = StateTransitionFcnQuadrotor6DOF(x, uState);
    
    // 创建过程噪声雅可比矩阵 (单位矩阵)
    MatrixXd W = MatrixXd::Identity(n, n);
    
    // 计算先验协方差矩阵
    P = J1 * P * J0.transpose() + W * Q * W.transpose();
}

//EKF校正函数
void SE3ControlNode::Correction(MatrixXd& P1, MatrixXd& J1,const VectorXd& x,const MatrixXd& Q,const VectorXd& uState, int n) {
 
    MatrixXd H_m = MatrixXd::Identity(12);
    MatrixXd V = MatrixXd::Identity(12);
    // 计算卡尔曼增益
    
    K = P1 * transpose(H_m)/(H_m*P1*transpose(H_m)+V*R*transpose(V));
    xnew = x1+K*(yMeas-x1);
}


void SE3ControlNode::JacobianMatrixQuadrotor6DOF(const VectorXd& x, const VectorXd& u) {
    const double dt = 0.001;
    const double g = 9.8;
    const double Jx = 6e-3;
    const double Jy = 2.1e-2;
    const double Jz = 2.5e-2;
    
    // 提取状态变量
    const double phi = x(6);   // Roll (x7)
    const double theta = x(7); // Pitch (x8)
    const double omega_x = x(9);  // Angular velocity x (x10)
    const double omega_y = x(10); // Angular velocity y (x11)
    const double omega_z = x(11); // Angular velocity z (x12)
    
    // 初始化雅可比矩阵
    MatrixXd Ak = MatrixXd::Zero(12, 12);
    
    // 位置导数部分
    Ak(0, 3) = 1; // dx/dvx
    Ak(1, 4) = 1; // dy/dvy
    Ak(2, 5) = 1; // dz/dvz
    
    // 速度导数部分 (重力相关)
    Ak(3, 7) = -g * cos(theta); // dvx/dtheta
    Ak(4, 6) = g * cos(phi) * cos(theta); // dvy/dphi
    Ak(4, 7) = -g * sin(phi) * sin(theta); // dvy/dtheta
    Ak(5, 6) = -g * sin(phi) * cos(theta); // dvz/dphi
    Ak(5, 7) = -g * cos(phi) * sin(theta); // dvz/dtheta
    
    // 欧拉角导数部分
    Ak(6, 6) = omega_y * cos(phi)*tan(theta) - omega_z * sin(phi)*tan(theta); // dphi/dphi
    Ak(6, 7) = (sin(phi)*omega_y + cos(phi)*omega_z) / pow(cos(theta), 2); // dphi/dtheta
    Ak(6, 10) = sin(phi)*tan(theta); // dphi/dwy
    Ak(6, 11) = cos(phi)*tan(theta); // dphi/dwz
    
    Ak(7, 6) = -omega_y * sin(phi) - omega_z * cos(phi); // dtheta/dphi
    Ak(7, 10) = cos(phi); // dtheta/dwy
    Ak(7, 11) = -sin(phi); // dtheta/dwz
    
    Ak(8, 6) = omega_y * cos(phi)/cos(theta) - omega_z * sin(phi)/cos(theta); // dpsi/dphi
    Ak(8, 7) = sin(theta)*(sin(phi)*omega_y + cos(phi)*omega_z) / pow(cos(theta), 2); // dpsi/dtheta
    Ak(8, 10) = sin(phi)/cos(theta); // dpsi/dwy
    Ak(8, 11) = cos(phi)/cos(theta); // dpsi/dwz
    
    // 角速度导数部分 (陀螺效应)
    Ak(9, 10) = -(Jz - Jy)*omega_z/Jx; // dwx/dwy
    Ak(9, 11) = -(Jz - Jy)*omega_y/Jx; // dwx/dwz
    
    Ak(10, 9) = -(Jx - Jz)*omega_z/Jy; // dwy/dwx
    Ak(10, 11) = -(Jx - Jz)*omega_x/Jy; // dwy/dwz
    
    Ak(11, 9) = -(Jy - Jx)*omega_y/Jz; // dwz/dwx
    Ak(11, 10) = -(Jy - Jx)*omega_x/Jz; // dwz/dwy
    
    // 计算离散时间雅可比矩阵
    MatrixXd J1 = MatrixXd::Identity(12, 12) + Ak * dt;
    return J1;
}

void SE3ControlNode::StateTransitionFcnQuadrotor6DOF(const VectorXd& x, const VectorXd& u) {
    const double mass = 1.19;
    const double dt = 0.001;
    const Vector3d g(0, 0, 9.8);
    
    // 提取状态变量
    const double phi = x(6);   // Roll (x7)
    const double theta = x(7); // Pitch (x8)
    const double psi = x(8);   // Yaw (x9)
    const Vector3d omega = x.segment(9, 3); // Angular velocity (x10-x12)
    
    // 体坐标系到惯性系的转换矩阵
    Matrix3d R;
    R << cos(theta)*cos(psi), 
         sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),
         cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi),
         
         cos(theta)*sin(psi),
         sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),
         cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi),
         
         -sin(theta),
         sin(phi)*cos(theta),
         cos(phi)*cos(theta);
    
    // 欧拉角导数矩阵
    Matrix3d E;
    E << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
         0, cos(phi),           -sin(phi),
         0, sin(phi)/cos(theta), cos(phi)/cos(theta);
    
    // 转动惯量矩阵及其逆
    Vector3d J_diag(6e-3, 2.1e-2, 2.5e-2);
    Matrix3d J = J_diag.asDiagonal();
    Matrix3d J_inv = (Vector3d(166.6666, 47.619, 40)).asDiagonal();
    
    // 计算加速度
    Vector3d force_body = u.head(3);
    Vector3d acceleration = R.transpose() * g - force_body / mass;
    
    // 计算欧拉角导数
    Vector3d Phi_dot = E * omega;
    
    // 计算角加速度
    Vector3d torque_body = u.tail(3);
    Vector3d Omega_dot = J_inv * (torque_body - omega.cross(J * omega));
    
    // 计算状态增量
    Vector3d DeltaX = x.segment(3, 3) * dt;      // 位移增量
    Vector3d DeltaV = acceleration * dt;         // 速度增量
    Vector3d DeltaPhi = Phi_dot * dt;            // 欧拉角增量
    Vector3d DeltaOmega = Omega_dot * dt;        // 角速度增量
    
    // 更新状态
    VectorXd xnew(12);
    xnew.segment(0, 3) = x.segment(0, 3) + DeltaX;      // 位置
    xnew.segment(3, 3) = x.segment(3, 3) + DeltaV;      // 速度
    xnew.segment(6, 3) = x.segment(6, 3) + DeltaPhi;    // 欧拉角
    xnew.segment(9, 3) = omega + DeltaOmega;            // 角速度
    
    return xnew;
}