#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Point control test node
        Node(
            package='px4ctrl_ros2',
            executable='point_control_test_node',
            name='point_control_test_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        
        # Optional: data monitor for debugging
        Node(
            package='px4ctrl_ros2',
            executable='data_monitor_node',
            name='data_monitor_node',
            output='screen',
            parameters=[]
        )
    ])