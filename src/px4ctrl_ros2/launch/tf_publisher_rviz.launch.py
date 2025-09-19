#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取RViz配置文件路径
    rviz_config_file_path = os.path.join(
        get_package_share_directory('px4ctrl_ros2'),
        'config', 'default.rviz'
    )
    
    return LaunchDescription([
        # UAV TF Publisher Node
        Node(
            package='px4ctrl_ros2',
            executable='uav_tf_publisher.py',
            name='uav_tf_publisher',
            output='screen',
            parameters=[
                {'uav_frame_id': 'base_link'},
                {'world_frame_id': 'map'},
                {'position_topic': '/fmu/out/vehicle_local_position'},
                {'attitude_topic': '/fmu/out/vehicle_attitude'}
            ]
        ),
        
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file_path]
        )
    ])