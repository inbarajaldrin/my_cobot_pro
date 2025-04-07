#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

import cv2
import numpy as np
import os
import json

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.joint_names = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
        ]

        self.current_joint_positions = {}
        self.default_z = 0.08
        self.joint_log = []
        self.output_file = os.path.expanduser('~/ros2_ws/src/my_cobot_pro/data/joint_trajectory_log.json')

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def detect_line_points(self, image_path, inter_num_points=10):
        image = cv2.imread(image_path)
        if image is None:
            self.get_logger().error(f"Could not load image: {image_path}")
            return []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        skeleton = cv2.ximgproc.thinning(binary)

        contours, _ = cv2.findContours(skeleton.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            self.get_logger().error("No contour found in image.")
            return []

        contour = max(contours, key=lambda c: cv2.arcLength(c, False))
        curve = contour[:, 0, :]

        num_points = inter_num_points * 2
        lengths = np.cumsum(np.sqrt(np.sum(np.diff(curve, axis=0) ** 2, axis=1)))
        lengths = np.insert(lengths, 0, 0.0)
        total_length = lengths[-1]
        even_spacing = np.linspace(0, total_length, num_points)

        sampled_points = []
        for dist in even_spacing:
            idx = np.searchsorted(lengths, dist)
            idx = min(idx, len(curve) - 2)
            p1, p2 = curve[idx], curve[idx + 1]
            segment_len = lengths[idx + 1] - lengths[idx]
            if segment_len == 0:
                sampled_points.append(tuple(p1))
                continue
            ratio = (dist - lengths[idx]) / segment_len
            interp = (1 - ratio) * p1 + ratio * p2
            sampled_points.append(tuple(map(int, interp)))

        x_dim = 0.15
        y_dim = 0.15
        img_h, img_w = image.shape[:2]

        converted_points = []
        for (px, py) in sampled_points:
            norm_x = px / img_w
            norm_y = 1.0 - (py / img_h)
            real_x = norm_x * x_dim
            real_y = norm_y * y_dim
            converted_points.append((real_x, real_y))

        cutoff = num_points // 2 + (num_points % 2)
        return converted_points[:cutoff]

    def publish_joint_trajectory(self, joint_positions, description=""):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 3
        traj.points.append(point)
        self.traj_pub.publish(traj)
        self.get_logger().info(f"Published: {description}")
        rclpy.spin_once(self, timeout_sec=1.0)

    def call_ik_and_move(self, x_input, y_input, z_value=None):
        if z_value is None:
            z_value = self.default_z

        x2 = 0.381
        y2 = 0.337
        x = float(x_input) - x2
        y = float(y_input) - y2

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped.header.frame_id = 'base'
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = float(z_value)

        req.ik_request.pose_stamped.pose.orientation.x = 0.5
        req.ik_request.pose_stamped.pose.orientation.y = -0.5
        req.ik_request.pose_stamped.pose.orientation.z = 0.5
        req.ik_request.pose_stamped.pose.orientation.w = 0.5

        try:
            current_positions = [self.current_joint_positions[name] for name in self.joint_names]
            req.ik_request.robot_state.joint_state.name = self.joint_names
            req.ik_request.robot_state.joint_state.position = current_positions
        except KeyError:
            self.get_logger().warn("Missing joint state for seeding IK. Proceeding without seed.")

        req.ik_request.timeout.sec = 2
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error("Service call failed or no result.")
            return None

        if future.result().error_code.val == 1:
            sol = future.result().solution
            joint_positions = list(sol.joint_state.position)

            # Apply joint 1 constraint (within +/- 90 degrees = +/- pi/2 radians)
            joint1 = joint_positions[0]
            if abs(joint1) > (np.pi / 2):
                self.get_logger().warn(f"Rejected solution: Joint 1 exceeds ±90° limit (value: {joint1:.3f} rad)")
                return None

            traj = JointTrajectory()
            traj.joint_names = sol.joint_state.name
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 3
            traj.points.append(point)
            self.traj_pub.publish(traj)
            self.joint_log.append(joint_positions)
            return joint_positions
        else:
            self.get_logger().error("IK solution not found.")
            return None

    def execute_line_follow(self, image_path, inter_num_points=10):
        home_position = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
        self.publish_joint_trajectory(home_position, "Home position")

        workspace_home = [0.704941, -2.266907, -2.281362, -0.19177, 1.567333, -0.22856]
        self.publish_joint_trajectory(workspace_home, "Workspace home position")

        points = self.detect_line_points(image_path, inter_num_points)
        if not points:
            self.get_logger().error("No valid points to execute.")
            return

        self.get_logger().info(f"Detected {len(points)} points along the line.")
        for i, (x, y) in enumerate(points):
            self.get_logger().info(f"Moving to point {i+1}/{len(points)}: x={x:.4f}, y={y:.4f}, z={self.default_z:.3f}")
            self.call_ik_and_move(x, y, self.default_z)
            rclpy.spin_once(self, timeout_sec=1.0)

        try:
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
            with open(self.output_file, 'w') as f:
                json.dump(self.joint_log, f, indent=2)
            self.get_logger().info(f"Saved {len(self.joint_log)} joint sets to: {self.output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to write JSON: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        image_path = "/home/aldrin/ros2_ws/src/my_cobot_pro/camera/lab3.png"
        node.execute_line_follow(image_path, inter_num_points=10)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
