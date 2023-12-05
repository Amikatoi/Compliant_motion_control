#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class CartesianMotionPublisher(Node):
    def __init__(self):
        super().__init__('cartesian_motion_publisher')

        self.controller_name = '/target_frame'

        # Publisher for Cartesian trajectories
        self.publisher = self.create_publisher(PoseStamped, self.controller_name, 10)

        # Define the robot's straight line motion in Cartesian space
        # These values should be determined by the desired end-effector path
        self.cartesian_goals = [
            {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
            {'x': 0.0, 'y': 0.2, 'z': 0.0, 'w': 0.0},
            # Add more Cartesian positions as needed
            #{'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        ]

        # Time in seconds required to move from one position to the next
        self.move_time = 2

    def publish_cartesian_goals(self):
        """ Publish predefined Cartesian trajectories to move the robot in a straight line. """
        for cartesian_goal in self.cartesian_goals:
            msg = PoseStamped()
            msg.pose.position.x = cartesian_goal['x']
            msg.pose.position.y = cartesian_goal['y']
            msg.pose.position.z = cartesian_goal['z']
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

