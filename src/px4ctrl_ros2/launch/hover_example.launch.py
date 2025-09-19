#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share_dir = get_package_share_directory('px4ctrl_ros2')
    
    # Path to the YAML config file
    config_file = '/root/FSM_ws/src/px4ctrl_ros2/config/ctrl_param_fpv.yaml'
    
    return LaunchDescription([
        # Hover example node with YAML parameters
        Node(
            package='px4ctrl_ros2',
            executable='hover_example_node',
            name='hover_example_node',  # 保持与代码中一致的节点名
            output='screen',
            arguments=['--ros-args', '--params-file', config_file],
            remappings=[]
        )
    ])