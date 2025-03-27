#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
import sys
import numpy as np

# Define camera workspace bounds in meters
x1 = -0.15  # Top-right X in robot base frame
x2 = -0.35  # Width of the workspace (extends left)
y1 = -0.15  # Top Y offset from robot base
y2 = -0.35  # Height of the workspace (extends downward)

# Define number of interpolation steps
interpolation_steps = 10

class IKCommander(Node):
    def __init__(self):
        super().__init__('ik_commander')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.current_position = None

    def move_to_camera_frame(self, x_cam, y_cam, z_target):
        # if not (0.0 <= x_cam <= -x2 and 0.0 <= y_cam <= -y2):
        #     self.get_logger().warn(f"Input ({x_cam}, {y_cam}) is out of camera workspace bounds.")
        #     return

        x_base = (x1 + x2) + x_cam
        y_base = y1 + y_cam

        self.get_logger().info(f"Transformed camera (x={x_cam}, y={y_cam}) to base (x={x_base}, y={y_base}), z={z_target}")
        self.move_to_pose_interpolated(x_base, y_base, z_target, steps=interpolation_steps)

    def move_to_pose_interpolated(self, x_target, y_target, z_target, steps=10):
        if self.current_position is None:
            self.current_position = (x_target, y_target, z_target)

        x_start, y_start, z_start = self.current_position
        xs = np.linspace(x_start, x_target, steps)
        ys = np.linspace(y_start, y_target, steps)
        zs = np.linspace(z_start, z_target, steps)

        tcp_commands = []

        for i in range(steps):
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'arm'
            request.ik_request.pose_stamped.header.frame_id = 'base'
            request.ik_request.pose_stamped.pose.position.x = float(xs[i])
            request.ik_request.pose_stamped.pose.position.y = float(ys[i])
            request.ik_request.pose_stamped.pose.position.z = float(zs[i])

            request.ik_request.pose_stamped.pose.orientation.x = -0.35
            request.ik_request.pose_stamped.pose.orientation.y = -0.62
            request.ik_request.pose_stamped.pose.orientation.z = -0.40
            request.ik_request.pose_stamped.pose.orientation.w =  0.57

            request.ik_request.timeout.sec = 1

            future = self.cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() and future.result().error_code.val == 1:
                joint_positions = future.result().solution.joint_state.position
                angles_deg = [round(np.degrees(pos), 3) for pos in joint_positions]
                speed = 500
                command_string = f"set_angles({', '.join(map(str, angles_deg))}, {speed})"
                tcp_commands.append(command_string)
                self.get_logger().info(f"[{i+1}/{steps}] ✅ IK success: {command_string}")
            else:
                self.get_logger().error(f"[{i+1}/{steps}] ❌ IK failed. Skipping this point.")

        self.get_logger().info("\n✅ Full TCP Command Sequence:\n")
        for cmd in tcp_commands:
            print(cmd)

        self.current_position = (x_target, y_target, z_target)

    def move_to_pose(self, x, y, z):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header.frame_id = 'base'
        request.ik_request.pose_stamped.pose.position.x = float(x)
        request.ik_request.pose_stamped.pose.position.y = float(y)
        request.ik_request.pose_stamped.pose.position.z = float(z)

        # request.ik_request.pose_stamped.pose.orientation.x = 0.0
        # request.ik_request.pose_stamped.pose.orientation.y = 0.0
        # request.ik_request.pose_stamped.pose.orientation.z = 0.0
        # request.ik_request.pose_stamped.pose.orientation.w = 1.0

        # Set orientation to rotate 90 degrees around X axis (EE Z points downward)
        request.ik_request.pose_stamped.pose.orientation.x = -0.35
        request.ik_request.pose_stamped.pose.orientation.y = -0.62
        request.ik_request.pose_stamped.pose.orientation.z = -0.40
        request.ik_request.pose_stamped.pose.orientation.w =  0.57

        request.ik_request.timeout.sec = 1

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().error_code.val == 1:
            joint_names = future.result().solution.joint_state.name
            joint_positions = future.result().solution.joint_state.position

            traj = JointTrajectory()
            traj.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 1
            traj.points.append(point)

            self.publisher.publish(traj)
            self.get_logger().info(f"Published trajectory to move to x={x}, y={y}, z={z} with 90° downward pitch")
        else:
            self.get_logger().error('IK solution not found or service call failed.')


def main(args=None):
    rclpy.init(args=args)
    node = IKCommander()

    try:
        while True:
            xyz = input("Enter camera-frame x, y, z (in meters, separated by space): ")
            if not xyz.strip():
                break
            x_cam, y_cam, z = map(float, xyz.strip().split())
            node.move_to_camera_frame(x_cam, y_cam, z)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
