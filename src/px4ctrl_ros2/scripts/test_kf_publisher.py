#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np

class KFPublisherTester(Node):
    def __init__(self):
        super().__init__('kf_publisher_tester')
        
        # 创建订阅者
        self.position_sub = self.create_subscription(
            PointStamped, '/kf_filtered/position', self.position_callback, 10)
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/kf_filtered/velocity', self.velocity_callback, 10)
        self.odometry_sub = self.create_subscription(
            Odometry, '/kf_filtered/odometry', self.odometry_callback, 10)
        self.state_sub = self.create_subscription(
            Float64MultiArray, '/kf_filtered/state_vector', self.state_callback, 10)
        self.covariance_sub = self.create_subscription(
            Float64MultiArray, '/kf_filtered/covariance', self.covariance_callback, 10)
            
        # 计数器
        self.position_count = 0
        self.velocity_count = 0
        self.odometry_count = 0
        self.state_count = 0
        self.covariance_count = 0
        
        # 创建定时器打印统计信息
        self.timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info("KF Publisher Tester started. Monitoring topics:")
        self.get_logger().info("  - /kf_filtered/position")
        self.get_logger().info("  - /kf_filtered/velocity")
        self.get_logger().info("  - /kf_filtered/odometry")
        self.get_logger().info("  - /kf_filtered/state_vector")
        self.get_logger().info("  - /kf_filtered/covariance")
    
    def position_callback(self, msg):
        self.position_count += 1
        if self.position_count % 20 == 1:  # 每20个消息打印一次
            self.get_logger().info(f"Position: x={msg.point.x:.3f}, y={msg.point.y:.3f}, z={msg.point.z:.3f}")
    
    def velocity_callback(self, msg):
        self.velocity_count += 1
        if self.velocity_count % 20 == 1:  # 每20个消息打印一次
            self.get_logger().info(f"Velocity: vx={msg.twist.linear.x:.3f}, vy={msg.twist.linear.y:.3f}, vz={msg.twist.linear.z:.3f}")
    
    def odometry_callback(self, msg):
        self.odometry_count += 1
        if self.odometry_count % 20 == 1:  # 每20个消息打印一次
            pos = msg.pose.pose.position
            vel = msg.twist.twist.linear
            self.get_logger().info(f"Odometry - Pos: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}], Vel: [{vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f}]")
    
    def state_callback(self, msg):
        self.state_count += 1
        if self.state_count % 20 == 1 and len(msg.data) >= 6:  # 每20个消息打印一次
            state = np.array(msg.data[:6])
            self.get_logger().info(f"State Vector: pos=[{state[0]:.3f}, {state[1]:.3f}, {state[2]:.3f}], vel=[{state[3]:.3f}, {state[4]:.3f}, {state[5]:.3f}]")
    
    def covariance_callback(self, msg):
        self.covariance_count += 1
        if self.covariance_count % 50 == 1:  # 每50个消息打印一次
            cov_data = np.array(msg.data).reshape(6, 6)
            # 打印对角线元素（方差）
            diagonal = np.diagonal(cov_data)
            self.get_logger().info(f"Covariance diagonal: [{diagonal[0]:.6f}, {diagonal[1]:.6f}, {diagonal[2]:.6f}, {diagonal[3]:.6f}, {diagonal[4]:.6f}, {diagonal[5]:.6f}]")
    
    def print_statistics(self):
        self.get_logger().info("=== KF Publisher Statistics ===")
        self.get_logger().info(f"Position messages: {self.position_count}")
        self.get_logger().info(f"Velocity messages: {self.velocity_count}")
        self.get_logger().info(f"Odometry messages: {self.odometry_count}")
        self.get_logger().info(f"State vector messages: {self.state_count}")
        self.get_logger().info(f"Covariance messages: {self.covariance_count}")
        
        if all([self.position_count > 0, self.velocity_count > 0, self.odometry_count > 0, 
                self.state_count > 0, self.covariance_count > 0]):
            self.get_logger().info("✅ All topics are publishing successfully!")
        else:
            missing = []
            if self.position_count == 0:
                missing.append("position")
            if self.velocity_count == 0:
                missing.append("velocity")
            if self.odometry_count == 0:
                missing.append("odometry")
            if self.state_count == 0:
                missing.append("state_vector")
            if self.covariance_count == 0:
                missing.append("covariance")
            self.get_logger().warn(f"❌ Missing topics: {', '.join(missing)}")

def main(args=None):
    rclpy.init(args=args)
    
    tester = KFPublisherTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Shutting down KF Publisher Tester...")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


