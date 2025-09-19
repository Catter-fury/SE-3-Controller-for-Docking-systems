#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径 - 使用标准的ROS2包路径方式
    config_file_path = os.path.join(
        get_package_share_directory('px4ctrl_ros2'),
        'config', 'ctrl_param_fpv.yaml'
    )
    
    return LaunchDescription([
        
        # PX4 Control FSM Node with parameters
        Node(
            package='px4ctrl_ros2',
            executable='px4ctrl_ros2_node',
            name='px4ctrl_fsm',
            output='screen',
            parameters=[config_file_path],  # 加载YAML参数文件
            remappings=[]
        ),

    ])