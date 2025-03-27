#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
import numpy as np

# Define camera workspace bounds in meters
x1 = -0.15  # Top-right X in robot base frame
x2 = -0.35  # Width of the workspace (extends left)
y1 = -0.15  # Top Y offset from robot base
y2 = -0.35  # Height of the workspace (extends downward)

# Define number of interpolation steps
interpolation_steps = 10


class CartesianCommander(Node):
    def __init__(self):
        super().__init__('cartesian_commander')

        self.cartesian_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        while not self.cartesian_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_cartesian_path service...')

        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.current_pose = None

    def move_to_camera_frame(self, x_cam, y_cam, z_target):
        x_base = (x1 + x2) + x_cam
        y_base = y1 + y_cam

        self.get_logger().info(f"Camera → Base: x={x_base:.3f}, y={y_base:.3f}, z={z_target:.3f}")
        self.send_cartesian_path(x_base, y_base, z_target, steps=interpolation_steps)

    def send_cartesian_path(self, x_target, y_target, z_target, steps=10):
        if self.current_pose is None:
            self.current_pose = (x_target, y_target, z_target)

        x_start, y_start, z_start = self.current_pose
        xs = np.linspace(x_start, x_target, steps)
        ys = np.linspace(y_start, y_target, steps)
        zs = np.linspace(z_start, z_target, steps)


        waypoints = []
        for i in range(steps):
            pose = Pose()
            pose.position.x = float(xs[i])
            pose.position.y = float(ys[i])
            pose.position.z = float(zs[i])
            pose.orientation.x = -0.3586897350156099
            pose.orientation.y = -0.6215518433721899
            pose.orientation.z = -0.407493211996637
            pose.orientation.w = 0.5647692114233287
            waypoints.append(pose)

        # Build service request
        request = GetCartesianPath.Request()
        request.group_name = 'arm'
        request.header.frame_id = 'base'
        request.waypoints = waypoints
        request.max_step = 0.01  # Max distance between waypoints (in meters)
        request.jump_threshold = 0.0  # Disable jump detection
        request.avoid_collisions = True

        # Call service
        future = self.cartesian_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().fraction > 0.0:
            traj = future.result().solution.joint_trajectory
            self.traj_pub.publish(traj)
            self.get_logger().info(
                f"Published trajectory — success fraction: {future.result().fraction:.2f}")
        else:
            self.get_logger().error("Cartesian path planning failed or returned zero-length trajectory.")

        self.current_pose = (x_target, y_target, z_target)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianCommander()

    try:
        while True:
            xyz = input("Enter camera-frame x, y, z (meters, space-separated): ")
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
