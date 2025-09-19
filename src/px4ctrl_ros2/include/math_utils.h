#pragma once
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// 从四元数计算 yaw（Z 轴朝上的右手系；与现有公式等价）
inline double quatToYaw(const Eigen::Quaterniond& q){
  return std::atan2(
      2.0 * (q.w()*q.z() + q.x()*q.y()),
      1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())
  );
}

// 从旋转矩阵计算 yaw（与当前 CSV 里使用的公式等价）
inline double rotMatToYaw(const Eigen::Matrix3d& R){
  return std::atan2(R(1,0), R(0,0));
}