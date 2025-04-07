#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK

import cv2
import numpy as np
import os
import json


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # 1) Set up client for IK
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        
        # 2) Publisher for joint trajectories
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Default height for IK motion
        self.default_z = 0.08

        # Storage for joint angles to save later
        self.joint_log = []

        # Output path for JSON log file
        self.output_file = os.path.expanduser('~/ros2_ws/src/my_cobot_pro/data/joint_trajectory_log.json')

    def detect_line_points(self, image_path, inter_num_points=10):
        image = cv2.imread(image_path)
        if image is None:
            self.get_logger().error(f"Could not load image: {image_path}")
            return []

        h, w = image.shape[:2]

        # Focus on a central crop of the image
        center_margin = 0.05  # 5% margin
        x1 = int(w * center_margin)
        x2 = int(w * (1 - center_margin))
        y1 = int(h * center_margin)
        y2 = int(h * (1 - center_margin))
        center_crop = image[y1:y2, x1:x2]

        # Convert to HSV for robust gray detection
        hsv_crop = cv2.cvtColor(center_crop, cv2.COLOR_BGR2HSV)

        # Define extended gray HSV range
        lower_hsv = np.array([0, 0, 30])
        upper_hsv = np.array([180, 70, 150])

        # Create and clean up mask
        mask = cv2.inRange(hsv_crop, lower_hsv, upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        # Skeletonize
        skeleton = cv2.ximgproc.thinning(mask)

        # Find contours
        contours, _ = cv2.findContours(skeleton.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            self.get_logger().error("No contours found in image.")
            return []

        # Choose longest contour
        contour = max(contours, key=lambda c: cv2.arcLength(c, False))
        curve = contour[:, 0, :]  # Nx2

        # Sample evenly spaced points along curve
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

        # Offset crop-relative points to original image coordinates
        sampled_points_original = [(pt[0] + x1, pt[1] + y1) for pt in sampled_points]

        # Normalize and convert to real-world coordinates
        x_dim = 0.15
        y_dim = 0.15
        img_h, img_w = image.shape[:2]

        converted_points = []
        for (px, py) in sampled_points_original:
            norm_x = px / img_w
            norm_y = 1.0 - (py / img_h)
            real_x = norm_x * x_dim
            real_y = norm_y * y_dim
            converted_points.append((real_x, real_y))

        # Only use half the points (to avoid overlap if bi-directional)
        cutoff = num_points // 2 + (num_points % 2)
        final_points = converted_points[:cutoff]
        return final_points

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

        # req.ik_request.pose_stamped.pose.orientation.x = 0.5
        # req.ik_request.pose_stamped.pose.orientation.y = -0.5
        # req.ik_request.pose_stamped.pose.orientation.z = 0.5
        # req.ik_request.pose_stamped.pose.orientation.w = 0.5

        req.ik_request.pose_stamped.pose.orientation.x = -0.21250174052234885
        req.ik_request.pose_stamped.pose.orientation.y = -0.6754919681652727
        req.ik_request.pose_stamped.pose.orientation.z = -0.23047410779884112
        req.ik_request.pose_stamped.pose.orientation.w = 0.667409392242503

        req.ik_request.timeout.sec = 2

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error("Service call failed or no result.")
            return None
        
        if future.result().error_code.val == 1:
            sol = future.result().solution
            joint_positions = list(sol.joint_state.position)
            joint_names = sol.joint_state.name

            self.get_logger().info(
                f"IK success at (x={x:.3f}, y={y:.3f}, z={z_value:.3f}). "
                f"Joints: {[f'{p:.3f}' for p in joint_positions]}"
            )

            # Publish to RViz / robot
            traj = JointTrajectory()
            traj.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 3
            traj.points.append(point)

            self.traj_pub.publish(traj)

            # Save to joint log
            self.joint_log.append(joint_positions)

            return joint_positions
        else:
            self.get_logger().error("IK solution not found.")
            return None

    def execute_line_follow(self, image_path, inter_num_points=10):
        points = self.detect_line_points(image_path, inter_num_points)
        if not points:
            self.get_logger().error("No valid points to execute.")
            return

        self.get_logger().info(f"Detected {len(points)} points along the line.")
        for i, (x, y) in enumerate(points):
            self.get_logger().info(f"Moving to point {i+1}/{len(points)}: x={x:.4f}, y={y:.4f}, z={self.default_z:.3f}")
            self.call_ik_and_move(x, y, self.default_z)
            rclpy.spin_once(self, timeout_sec=1.0)

        # Save all joint positions to JSON
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
        image_path = "/home/aldrin/ros2_ws/src/my_cobot_pro/camera/lab3.jpg"
        node.execute_line_follow(image_path, inter_num_points=5)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
