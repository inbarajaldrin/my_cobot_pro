#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK

class IKCommander(Node):
    def __init__(self):
        super().__init__('ik_commander')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    # def move_to_pose(self, x, y, z):
    #     request = GetPositionIK.Request()
    #     request.ik_request.group_name = 'arm'
    #     request.ik_request.pose_stamped.header.frame_id = 'base'
    #     request.ik_request.pose_stamped.pose.position.x = float(x)
    #     request.ik_request.pose_stamped.pose.position.y = float(y)
    #     request.ik_request.pose_stamped.pose.position.z = float(z)

    #     # Fixed orientation quaternion
    #     # request.ik_request.pose_stamped.pose.orientation.x = 0.4036051660577681
    #     # request.ik_request.pose_stamped.pose.orientation.y = -0.5710728135086158
    #     # request.ik_request.pose_stamped.pose.orientation.z = 0.35068965214700704
    #     # request.ik_request.pose_stamped.pose.orientation.w = 0.6228928314564606

    #     request.ik_request.pose_stamped.pose.orientation.x = 0.5
    #     request.ik_request.pose_stamped.pose.orientation.y = -0.5
    #     request.ik_request.pose_stamped.pose.orientation.z = 0.5
    #     request.ik_request.pose_stamped.pose.orientation.w = 0.5

    #     request.ik_request.timeout.sec = 2

    #     future = self.cli.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)

    #     if future.result() and future.result().error_code.val == 1:
    #         joint_names = future.result().solution.joint_state.name
    #         joint_positions = future.result().solution.joint_state.position

    #         traj = JointTrajectory()
    #         traj.joint_names = joint_names
    #         point = JointTrajectoryPoint()
    #         point.positions = joint_positions
    #         point.time_from_start.sec = 3
    #         traj.points.append(point)

    #         self.publisher.publish(traj)
    #         self.get_logger().info(f"IK successful. Moving to x={x}, y={y}, z={z} with fixed orientation")
    #         self.get_logger().info(f"Joint positions: {[f'{p:.3f}' for p in joint_positions]}")
    #     else:
    #         self.get_logger().error('IK solution not found or service call failed.')

    def move_to_pose(self, x, y, z):
        GROUND_Z_LIMIT = 0.0  # Set your ground level

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        # request.ik_request.avoid_collisions = True
        request.ik_request.pose_stamped.header.frame_id = 'base'
        request.ik_request.pose_stamped.pose.position.x = float(x)
        request.ik_request.pose_stamped.pose.position.y = float(y)
        request.ik_request.pose_stamped.pose.position.z = float(z)

        request.ik_request.pose_stamped.pose.orientation.x = 0.5
        request.ik_request.pose_stamped.pose.orientation.y = -0.5
        request.ik_request.pose_stamped.pose.orientation.z = 0.5
        request.ik_request.pose_stamped.pose.orientation.w = 0.5

        request.ik_request.timeout.sec = 2

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().error_code.val == 1:
            ik_pose = request.ik_request.pose_stamped.pose
            if ik_pose.position.z < GROUND_Z_LIMIT:
                self.get_logger().error(f"Rejected motion: z={ik_pose.position.z:.3f} below ground limit ({GROUND_Z_LIMIT}).")
                return

            joint_names = future.result().solution.joint_state.name
            joint_positions = future.result().solution.joint_state.position

            traj = JointTrajectory()
            traj.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 3
            traj.points.append(point)

            self.publisher.publish(traj)
            self.get_logger().info(f"IK successful. Moving to x={x}, y={y}, z={z} with fixed orientation")
            self.get_logger().info(f"Joint positions: {[f'{p:.3f}' for p in joint_positions]}")
        else:
            self.get_logger().error('IK solution not found or service call failed.')

def main(args=None):
    rclpy.init(args=args)
    node = IKCommander()

    try:
        while True:
            inp = input("Enter target x y z (separated by spaces): ")
            if not inp.strip():
                break
            x, y, z = inp.strip().split()
            node.move_to_pose(x, y, z)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
