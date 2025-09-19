# SE3控制器定点悬停案例

这个案例展示了如何使用基于SE(3)理论的控制器实现四旋翼的定点悬停功能。

## 功能特点

- **精确位置控制**: 使用SE(3)几何控制理论实现高精度位置控制
- **稳定姿态控制**: 基于旋转矩阵的姿态控制，避免奇点问题  
- **实时控制**: 50Hz控制频率，确保响应速度和稳定性
- **参数可配置**: 通过YAML文件配置控制增益和物理参数
- **状态监控**: 实时显示位置、姿态、控制输出等信息

## 核心技术

### SE(3)控制理论
- **几何位置控制**: 基于位置和速度误差的几何控制器
- **几何姿态控制**: 基于旋转矩阵误差的姿态控制器
- **导数计算**: 准确计算期望旋转矩阵的时间导数
- **积分控制**: 可选的积分控制用于消除稳态误差

### 控制架构
```
传感器数据 → SE3控制器 → 推力/力矩指令 → PX4
    ↑                           ↓
    └── 状态反馈 ←── 位置/姿态估计 ←┘
```

## 文件结构

```
src/
├── hover_example.cpp          # 主要的悬停控制节点
├── se3con.cpp                # SE3控制器实现
└── se3con.hpp                # SE3控制器头文件

config/
└── ctrl_param_fpv.yaml       # 控制参数配置文件

test_hover_example.sh         # 快速测试脚本
```

## 使用方法

### 1. 编译
```bash
cd /root/FSM_ws
colcon build --packages-select px4ctrl_ros2
source install/setup.bash
```

### 2. 运行悬停示例
```bash
# 方法1: 使用提供的测试脚本
./test_hover_example.sh

# 方法2: 直接运行节点
ros2 run px4ctrl_ros2 hover_example_node --ros-args --params-file src/px4ctrl_ros2/config/ctrl_param_fpv.yaml
```

### 3. 监控状态
程序会每1秒输出一次状态信息，包括：
- 当前位置和目标位置
- 位置误差和速度
- 姿态角度(Roll/Pitch/Yaw)
- 控制输出(推力和力矩)
- 控制状态(STABILIZING/CONVERGING/HOVERING)

## 参数配置

主要控制参数在 `config/ctrl_param_fpv.yaml` 中配置：

### 位置控制增益
```yaml
"se3.Kp_x": 2.5    # X轴位置增益
"se3.Kp_y": 2.5    # Y轴位置增益  
"se3.Kp_z": 4.0    # Z轴位置增益
"se3.Kv_x": 3.0    # X轴速度增益
"se3.Kv_y": 3.0    # Y轴速度增益
"se3.Kv_z": 4.5    # Z轴速度增益
```

### 姿态控制增益
```yaml
"se3.Kr_x": 0.8    # Roll姿态增益
"se3.Kr_y": 0.8    # Pitch姿态增益
"se3.Kr_z": 0.4    # Yaw姿态增益
"se3.Kw_x": 0.3    # Roll角速度增益
"se3.Kw_y": 0.3    # Pitch角速度增益
"se3.Kw_z": 0.15   # Yaw角速度增益
```

### 物理参数
```yaml
"se3.mass": 1.2           # 无人机质量(kg)
"se3.gravity": 9.81       # 重力加速度(m/s²)
"se3.Ixx": 0.02          # X轴转动惯量
"se3.Iyy": 0.02          # Y轴转动惯量
"se3.Izz": 0.04          # Z轴转动惯量
"thr_map.hover_percentage": 0.47  # 悬停油门百分比
```

## 代码结构说明

### HoverExampleNode类
主要的ROS2节点类，负责：
- 订阅PX4传感器数据(位置、姿态、角速度)
- 调用SE3控制器计算控制输出
- 发布推力和力矩控制指令
- 状态监控和日志输出

### 关键函数

#### `createHoverDesiredState()`
创建悬停期望状态：
```cpp
des.p = target_position_;           // 期望位置[0,0,-2]
des.v = Eigen::Vector3d::Zero();    // 期望速度为0
des.a = Eigen::Vector3d::Zero();    // 期望加速度为0  
des.yaw = target_yaw_;              // 期望偏航角为0
des.yaw_rate = 0.0;                 // 期望偏航角速度为0
```

#### `controlLoop()`
50Hz控制循环：
1. 检查传感器数据有效性
2. 转换当前状态格式
3. 创建期望状态
4. 调用SE3控制器计算输出
5. 发布控制指令
6. 状态监控和日志

## 预期行为

### 启动阶段 (0-5秒)
- 等待传感器数据
- 初始化SE3控制器
- 显示参数配置信息

### 控制阶段 (5秒后)
- 发送ARM和OFFBOARD指令
- 开始闭环位置控制
- 无人机应平稳上升到目标高度(2米)并保持悬停

### 悬停状态
- 位置误差 < 10cm
- 速度 < 5cm/s  
- 姿态稳定，偏航角保持0度
- 状态显示"HOVERING"

## 故障排除

### 1. 编译错误
确保安装了所有依赖：
```bash
sudo apt install ros-humble-eigen3-cmake-module
sudo apt install libeigen3-dev
```

### 2. 运行时错误
- 检查PX4仿真是否正在运行
- 确认话题名称正确(`/fmu/out/*`, `/fmu/in/*`)
- 检查参数文件路径是否正确

### 3. 控制不稳定
- 检查质量和惯性参数是否与实际无人机匹配
- 适当调整控制增益，从较小值开始
- 确认悬停推力值(0.47)是否合适

### 4. 位置漂移
- 检查位置估计是否准确
- 考虑启用积分控制(`se3.use_integral: true`)
- 检查传感器标定和融合算法

## 扩展功能

基于此悬停案例，可以进一步开发：

1. **轨迹跟踪**: 替换固定悬停点为时变轨迹
2. **航点导航**: 实现多个航点之间的平滑飞行
3. **外部扰动补偿**: 增强鲁棒性应对风扰等干扰
4. **自适应控制**: 根据飞行条件自动调整控制参数
5. **安全保护**: 添加围栏、电量监控等安全功能

## 理论背景

此控制器基于以下文献：
- Lee, T., Leok, M., & McClamroch, N. H. (2010). "Geometric tracking control of a quadrotor UAV on SE(3)"
- 使用Special Euclidean Group SE(3)上的几何控制理论
- 避免了传统欧拉角方法的奇点问题
- 保证了全局渐近稳定性

## 许可证

本代码遵循项目许可证条款。