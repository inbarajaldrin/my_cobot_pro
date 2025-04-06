#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene

class GroundPlaneAdder(Node):
    def __init__(self):
        super().__init__('ground_plane_adder')

        self.scene_srv = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.scene_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for apply_planning_scene service...")

        self.add_ground_plane()

    def add_ground_plane(self):
        ground = CollisionObject()
        ground.id = "ground_plane"
        ground.header.frame_id = "base"

        # Define the primitive (a large thin box)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.01]  # 2x2 meters, 1cm thick

        # Define pose of the box (centered at z = -0.005)
        pose = Pose()
        pose.position.z = -0.005  # half the thickness below base link

        ground.primitives = [box]
        ground.primitive_poses = [pose]
        ground.operation = CollisionObject.ADD

        # Wrap in planning scene message
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [ground]

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.scene_srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info("Ground plane added to planning scene.")
        else:
            self.get_logger().error("Failed to add ground plane.")

def main(args=None):
    rclpy.init(args=args)
    node = GroundPlaneAdder()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
