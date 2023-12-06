#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class CartesianMotionPublisher(Node):
    def __init__(self):
        super().__init__('cartesian_motion_publisher')

        # The cartesian motion controller listens on '/target_frame'
        self.controller_name = '/target_frame'

        # Publisher for Cartesian trajectories
        self.publisher = self.create_publisher(PoseStamped, self.controller_name, 10)

        # Define the robot's motion in Cartesian space
        self.cartesian_goals = [
            {'x': 0.81725, 'y': 0.2353, 'z': 0.1, 'wx': -1.57e-16, 'wy': 0.707, 'wz': 0.707, 'w': 1.57e-16},
            {'x': 0.81725, 'y': 0.2353, 'z': 0.2, 'wx': -1.57e-16, 'wy': 0.707, 'wz': 0.707, 'w': 1.57e-16},
            {'x': 0.71725, 'y': 0.2353, 'z': 0.2, 'wx': -1.57e-16, 'wy': 0.707, 'wz': 0.707, 'w': 1.57e-16},
            {'x': 0.71725, 'y': 0.2353, 'z': 0.1, 'wx': -1.57e-16, 'wy': 0.707, 'wz': 0.707, 'w': 1.57e-16},
            {'x': 0.81725, 'y': 0.2353, 'z': 0.1, 'wx': -1.57e-16, 'wy': 0.707, 'wz': 0.707, 'w': 1.57e-16},
            {'x': 0.81725, 'y': 0.2353, 'z': 0.0, 'wx': -1.57e-16, 'wy': 0.707, 'wz': 0.707, 'w': 1.57e-16},
        ]

        # Time in seconds required to move from one position to the next
        self.move_time = 3

    def publish_cartesian_goals(self):
        """ Publish predefined Cartesian trajectories to move the robot in a straight line. """
        for cartesian_goal in self.cartesian_goals:
            msg = PoseStamped()
            msg.header.frame_id = "base_link"
            msg.pose.position.x = cartesian_goal['x']
            msg.pose.position.y = cartesian_goal['y']
            msg.pose.position.z = cartesian_goal['z']
            msg.pose.orientation.x = cartesian_goal['wx']
            msg.pose.orientation.y = cartesian_goal['wy']
            msg.pose.orientation.z = cartesian_goal['wz']
            msg.pose.orientation.w = cartesian_goal['w']

            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=self.move_time)
            self.get_logger().info(f'Published Cartesian goal: {cartesian_goal}')


def main(args=None):
    rclpy.init(args=args)
    cartesian_motion_publisher = CartesianMotionPublisher()
    cartesian_motion_publisher.publish_cartesian_goals()
    cartesian_motion_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

