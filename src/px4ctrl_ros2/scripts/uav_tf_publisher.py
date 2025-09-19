#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
import numpy as np


class UAVTFPublisher(Node):
    def __init__(self):
        super().__init__('uav_tf_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.current_position = None
        self.current_orientation = None
        self.debug_counter = 0
        
        self.declare_parameter('uav_frame_id', 'base_link')
        self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('position_topic', '/fmu/out/vehicle_local_position')
        self.declare_parameter('attitude_topic', '/fmu/out/vehicle_attitude')
        
        self.uav_frame_id = self.get_parameter('uav_frame_id').get_parameter_value().string_value
        self.world_frame_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        position_topic = self.get_parameter('position_topic').get_parameter_value().string_value
        attitude_topic = self.get_parameter('attitude_topic').get_parameter_value().string_value
        
        # Configure QoS to match PX4's DDS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            position_topic,
            self.position_callback,
            qos_profile
        )
        
        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            attitude_topic,
            self.attitude_callback,
            qos_profile
        )
        
        self.timer = self.create_timer(0.02, self.publish_transforms)
        
        # Timer for static transform publishing (every 1 second)
        self.static_timer = self.create_timer(1.0, self.publish_static_transforms)
        
        self.get_logger().info(f'UAV TF Publisher initialized for PX4 DDS with NED to ENU conversion')
        self.get_logger().info(f'Listening to position topic: {position_topic}')
        self.get_logger().info(f'Listening to attitude topic: {attitude_topic}')
        self.get_logger().info(f'Publishing transforms: {self.world_frame_id} -> {self.uav_frame_id} (ENU coordinate system)')
        self.get_logger().info(f'Publishing static transforms:')
        self.get_logger().info(f'  {self.uav_frame_id} -> {self.camera_frame_id}')

    def position_callback(self, msg):
        # PX4 VehicleLocalPosition is in NED coordinate system
        # Convert NED to ENU coordinate system for ROS standard
        position = type('Position', (), {})()
        position.x = msg.y   # East (PX4 East -> ROS X)
        position.y = msg.x   # North (PX4 North -> ROS Y)
        position.z = -msg.z  # Up (PX4 Down -> ROS Up, negate Z)
        
        self.current_position = position
        

    def attitude_callback(self, msg):
        # PX4 quaternion is in NED frame (FRD body frame relative to NED world frame)
        # Convert from NED to ENU coordinate system
        # NED to ENU quaternion conversion: rotate 180° around Z, then 90° around Y
        orientation = type('Orientation', (), {})()
        
        # PX4 quaternion (NED frame)
        qw_ned = msg.q[0]
        qx_ned = msg.q[1]
        qy_ned = msg.q[2]
        qz_ned = msg.q[3]
        
        # Convert NED quaternion to ENU quaternion
        # This transformation accounts for the coordinate system change
        orientation.w = qw_ned    # qw remains the same
        orientation.x = qy_ned    # qx_enu = qy_ned
        orientation.y = qx_ned    # qy_enu = qx_ned  
        orientation.z = -qz_ned   # qz_enu = -qz_ned
        
        self.current_orientation = orientation

    def publish_static_transforms(self):
        """Publish all static transforms"""
        # Static transforms should use current time for proper TF tree connectivity
        now = self.get_clock().now().to_msg()
        
        # base_link to camera_link transform
        camera_transform = TransformStamped()
        camera_transform.header.stamp = now
        camera_transform.header.frame_id = self.uav_frame_id
        camera_transform.child_frame_id = self.camera_frame_id
        
        # Camera position relative to base_link: (0.1, 0, 0.25, 0, 0, 0)
        camera_transform.transform.translation.x = 0.1
        camera_transform.transform.translation.y = 0.0
        camera_transform.transform.translation.z = 0.25
        camera_transform.transform.rotation.x = 0.0
        camera_transform.transform.rotation.y = 0.0
        camera_transform.transform.rotation.z = 0.0
        camera_transform.transform.rotation.w = 1.0
        
        # Send static transform
        self.static_tf_broadcaster.sendTransform(camera_transform)

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()
        
        # Debug output every 5 seconds (250 calls at 50Hz)
        self.debug_counter += 1
        if self.debug_counter % 250 == 0:
            self.get_logger().info(f'TF Status: Position={self.current_position is not None}, '
                                 f'Orientation={self.current_orientation is not None}, '
                                 f'Publishing map->base_link->camera_link')
        
        if self.current_position is not None:
            uav_transform = TransformStamped()
            uav_transform.header.stamp = now
            uav_transform.header.frame_id = self.world_frame_id
            uav_transform.child_frame_id = self.uav_frame_id
            
            uav_transform.transform.translation.x = float(self.current_position.x)
            uav_transform.transform.translation.y = float(self.current_position.y)
            uav_transform.transform.translation.z = float(self.current_position.z)
            
            if self.current_orientation is not None:
                uav_transform.transform.rotation.x = float(self.current_orientation.x)
                uav_transform.transform.rotation.y = float(self.current_orientation.y)
                uav_transform.transform.rotation.z = float(self.current_orientation.z)
                uav_transform.transform.rotation.w = float(self.current_orientation.w)
            else:
                # Use identity quaternion if no orientation data
                uav_transform.transform.rotation.x = 0.0
                uav_transform.transform.rotation.y = 0.0
                uav_transform.transform.rotation.z = 0.0
                uav_transform.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(uav_transform)


def main(args=None):
    rclpy.init(args=args)
    
    uav_tf_publisher = UAVTFPublisher()
    
    try:
        rclpy.spin(uav_tf_publisher)
    except KeyboardInterrupt:
        pass
    
    uav_tf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()