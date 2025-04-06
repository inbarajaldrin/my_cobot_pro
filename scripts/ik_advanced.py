#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from tf_transformations import quaternion_from_euler  # Use this instead of transforms3d
import math

class IKCommander(Node):
    def __init__(self):
        super().__init__('ik_commander')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def move_to_pose(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        # Convert degrees to radians
        rx = math.radians(float(roll_deg))
        ry = math.radians(float(pitch_deg))
        rz = math.radians(float(yaw_deg))

        # Convert RPY to quaternion (intrinsic ZYX = standard for ROS)
        q = quaternion_from_euler(rx, ry, rz)

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header.frame_id = 'base'
        request.ik_request.pose_stamped.pose.position.x = float(x)
        request.ik_request.pose_stamped.pose.position.y = float(y)
        request.ik_request.pose_stamped.pose.position.z = float(z)

        request.ik_request.pose_stamped.pose.orientation.x = q[0]
        request.ik_request.pose_stamped.pose.orientation.y = q[1]
        request.ik_request.pose_stamped.pose.orientation.z = q[2]
        request.ik_request.pose_stamped.pose.orientation.w = q[3]

        request.ik_request.timeout.sec = 2

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().error_code.val == 1:
            joint_names = future.result().solution.joint_state.name
            joint_positions = future.result().solution.joint_state.position

            traj = JointTrajectory()
            traj.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 3
            traj.points.append(point)

            self.publisher.publish(traj)
            self.get_logger().info(f"IK successful. Moving to x={x}, y={y}, z={z} with RPY={roll_deg}, {pitch_deg}, {yaw_deg} degrees")
            self.get_logger().info(f"Joint positions: {[f'{p:.3f}' for p in joint_positions]}")
        else:
            self.get_logger().error('IK solution not found or service call failed.')

def main(args=None):
    rclpy.init(args=args)
    node = IKCommander()

    try:
        while True:
            inp = input("Enter target x y z roll pitch yaw (in DEGREES, separated by spaces): ")
            if not inp.strip():
                break
            x, y, z, roll, pitch, yaw = inp.strip().split()
            node.move_to_pose(x, y, z, roll, pitch, yaw)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
