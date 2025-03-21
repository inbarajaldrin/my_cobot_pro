#!/usr/bin/env python3
import rclpy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose

def main():
    rclpy.init()

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("arm")  # Replace with your group name

    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x = 0.4
    pose.position.y = 0.0
    pose.position.z = 0.4

    group.set_pose_target(pose)

    plan = group.plan()
    if plan[0]:  # Check if planning succeeded
        print("Planning succeeded, executing...")
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
    else:
        print("Planning failed.")

if __name__ == '__main__':
    main()
