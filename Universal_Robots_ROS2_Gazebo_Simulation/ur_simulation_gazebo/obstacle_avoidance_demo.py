import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        # Publisher to the planning scene
        self.publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)

    def add_obstacle(self):
        # --- Define Shared Primitive (Size) ---
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.1, 0.1, 0.8] # 0.2m x 0.2m x 1.0m tall

        # --- Box 1 Setup ---
        box1 = CollisionObject()
        box1.id = "box_obstacle1"
        box1.header.frame_id = "base_link"
        
        # Pose for Box 1 (y = -0.3)
        pose1 = PoseStamped()
        pose1.header.frame_id = "base_link"
        pose1.pose.position.x = 0.1
        pose1.pose.position.y = 0.3
        pose1.pose.position.z = 0.4 # Lifted up slightly so it sits on floor (assuming z=0 is floor)
        pose1.pose.orientation.w = 1.0

        box1.primitives.append(primitive)
        box1.primitive_poses.append(pose1.pose)
        box1.operation = CollisionObject.ADD

        # --- Box 2 Setup ---
        box2 = CollisionObject()
        box2.id = "box_obstacle2"
        box2.header.frame_id = "base_link"

        # Pose for Box 2 (y = +0.3) - Distinct location
        pose2 = PoseStamped()
        pose2.header.frame_id = "base_link"
        pose2.pose.position.x = 0.1
        pose2.pose.position.y = -0.3
        pose2.pose.position.z = 0.4
        pose2.pose.orientation.w = -1.0

        box2.primitives.append(primitive)
        box2.primitive_poses.append(pose2.pose)
        box2.operation = CollisionObject.ADD

        # --- Publish Both ---
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        # Add both boxes to the same list
        scene_msg.world.collision_objects.append(box1)
        scene_msg.world.collision_objects.append(box2)

        self.publisher.publish(scene_msg)
        self.get_logger().info('Published 2 obstacles to planning scene')

def main():
    rclpy.init()
    node = ObstaclePublisher()
    # Wait briefly for connections then publish
    rclpy.spin_once(node, timeout_sec=2.0)
    node.add_obstacle()
    rclpy.shutdown()

if __name__ == '__main__':
    main()