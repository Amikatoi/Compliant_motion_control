#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class CartesianPublisher(Node):
    def __init__(self):
        super().__init__('cartesian_publisher')

        # Define the topic to publish PoseStamped messages
        self.topic_name = self.declare_parameter('topic_name', '/target_frame').value

        # Publisher for PoseStamped messages
        self.publisher = self.create_publisher(PoseStamped, self.topic_name, 10)

        # Predefined Cartesian coordinates
        self.cartesian_poses = [
            {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 0},
            {'x': 0.0, 'y': 0.2, 'z': 0.0, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 0},
            {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 0},
            # Add more poses as needed
        ]

        # Timer to publish poses at regular intervals
        self.timer = self.create_timer(2.0, self.publish_pose)

    def publish_pose(self):
        if not self.cartesian_poses:
            self.get_logger().info('No more poses to publish')
            return

        # Get the next pose
        pose_data = self.cartesian_poses.pop(0)

        # Create a PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_frame"  # Set the appropriate frame ID
        msg.pose.position.x = float(pose_data['x'])
        msg.pose.position.y = float(pose_data['y'])
        msg.pose.position.z = float(pose_data['z'])
        msg.pose.orientation.x = float(pose_data['qx'])
        msg.pose.orientation.y = float(pose_data['qy'])
        msg.pose.orientation.z = float(pose_data['qz'])
        msg.pose.orientation.w = float(pose_data['qw'])

        # Publish the PoseStamped message
        self.publisher.publish(msg)
        self.get_logger().info(f"Published pose: {msg.pose}")

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
