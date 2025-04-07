#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import json
import os
import math


class TrajectoryReplayer(Node):
    def __init__(self):
        super().__init__('trajectory_replayer')

        # Joint names
        self.joint_names = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]

        # Publisher
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Subscriber
        self.current_joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Load joint trajectory
        self.input_file = os.path.expanduser('~/ros2_ws/src/my_cobot_pro/data/joint_trajectory_log.json')
        self.joint_trajectory_points = self.load_joint_trajectory()

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def load_joint_trajectory(self):
        if not os.path.exists(self.input_file):
            self.get_logger().error(f"Joint file not found: {self.input_file}")
            return []

        try:
            with open(self.input_file, 'r') as f:
                data = json.load(f)
                self.get_logger().info(f"Loaded {len(data)} joint sets from {self.input_file}")
                return data
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
            return []

    def publish_joint_trajectory(self, positions, description):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 3
        traj.points.append(point)

        self.publisher.publish(traj)
        self.get_logger().info(f"Published: {description}")
        self.wait_until_reached(positions)

    # def wait_until_reached(self, target_positions, tolerance=0.05, timeout=10.0):
    #     self.get_logger().info("Waiting for joint positions to reach target...")
    #     start_time = self.get_clock().now().nanoseconds / 1e9

    #     while rclpy.ok():
    #         # Fetch joint values in the correct order
    #         try:
    #             current_positions = [self.current_joint_positions[name] for name in self.joint_names]
    #         except KeyError as missing:
    #             self.get_logger().warn(f"Waiting for joint: {missing}")
    #             rclpy.spin_once(self, timeout_sec=0.1)
    #             continue

    #         # Compare with target
    #         errors = [abs(c - t) for c, t in zip(current_positions, target_positions)]
    #         self.get_logger().info(f"Joint errors: {[round(e, 4) for e in errors]}")

    #         if all(e <= tolerance for e in errors):
    #             self.get_logger().info("Target joint positions reached.")
    #             return

    #         elapsed = self.get_clock().now().nanoseconds / 1e9 - start_time
    #         if elapsed > timeout:
    #             self.get_logger().warn("Timeout waiting for joint positions to reach target.")
    #             return

    #         rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until_reached(self, target_positions, tolerance=0.05, timeout=10.0):
        start_time = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():
            try:
                current_positions = [self.current_joint_positions[name] for name in self.joint_names]
            except KeyError:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            if all(abs(c - t) <= tolerance for c, t in zip(current_positions, target_positions)):
                return

            if self.get_clock().now().nanoseconds / 1e9 - start_time > timeout:
                return

            rclpy.spin_once(self, timeout_sec=0.1)

    def replay_trajectory(self):
        if not self.joint_trajectory_points:
            self.get_logger().warn("No joint trajectory points to replay.")
            return

        # Step 1: Send to home position
        home_position = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
        self.publish_joint_trajectory(home_position, "Home position")

        # Step 2: Send to workspace home
        workspace_home_position = [0.704941, -2.266907, -2.281362, -0.19177, 1.567333, -0.22856]
        self.publish_joint_trajectory(workspace_home_position, "Workspace home position")

        # Step 3: Send points from the file
        for i, joint_positions in enumerate(self.joint_trajectory_points):
            self.publish_joint_trajectory(joint_positions, f"Trajectory point {i+1}/{len(self.joint_trajectory_points)}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayer()
    try:
        node.replay_trajectory()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
