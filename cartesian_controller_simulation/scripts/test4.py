#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')

        # We assume the joint trajectory controller is named /joint_trajectory_controller.
        self.controller_name = '/joint_trajectory_controller'

        # Publisher for joint trajectories.
        self.publisher = self.create_publisher(
            JointTrajectory, f'{self.controller_name}/joint_trajectory', 10)

        # Configuration presets for your robot's joint names and goal positions.
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
        ]
        
        self.initial_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Define your robot's straight line motion in joint space.
        # These values should be determined by the desired end-effector path.
        self.straight_line_goals = [
            [0.0, -np.pi/4, np.pi/2, 0.0, np.pi/4, 0.0],
            [0.0, -np.pi/4, np.pi/2, 0.0, np.pi/4, np.pi/2],
            # Add more joint positions as needed to form a straight line
            self.initial_joint_positions
        ]

        # Time in seconds required to move from one position to the next.
        self.move_time = 2

    def publish_joint_goals(self):
        """ Publish predefined joint trajectories to move the robot in a straight line. """
        for joint_goal in self.straight_line_goals:
            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            jtp = JointTrajectoryPoint()
            jtp.positions = joint_goal
            jtp.time_from_start = Duration(sec=self.move_time)
            msg.points.append(jtp)
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=self.move_time)
            self.get_logger().info(f'Published joint goal: {joint_goal}')


def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    joint_trajectory_publisher.publish_joint_goals()
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

