#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
import sys

class IKCommander(Node):
    def __init__(self):
        super().__init__('ik_commander')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def move_to_pose(self, x, y, z):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'  # Replace with your planning group name
        request.ik_request.pose_stamped.header.frame_id = 'base'  # Replace with your robot's base frame
        request.ik_request.pose_stamped.pose.position.x = float(x)
        request.ik_request.pose_stamped.pose.position.y = float(y)
        request.ik_request.pose_stamped.pose.position.z = float(z)
        request.ik_request.pose_stamped.pose.orientation.w = 1.0
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
            point.time_from_start.sec = 3
            traj.points.append(point)

            self.publisher.publish(traj)
            self.get_logger().info(f"Published trajectory to move to x={x}, y={y}, z={z}")
        else:
            self.get_logger().error('IK solution not found or service call failed.')


def main(args=None):
    rclpy.init(args=args)
    node = IKCommander()

    try:
        while True:
            xyz = input("Enter target x, y, z (separated by spaces): ")
            if not xyz.strip():
                break
            x, y, z = xyz.strip().split()
            node.move_to_pose(x, y, z)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
