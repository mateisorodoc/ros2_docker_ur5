import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math
import numpy as np

# --- UR5e PARAMETERS ---
# D (Link Offsets) and A (Link Lengths)
d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
a = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]

def inverse_kinematics_ur5e(x, y, z, roll, pitch, yaw):
    """
    Calculates the 6 joint angles for the UR5e to reach (x,y,z) with (r,p,y).
    Returns the 'Elbow Up' solution.
    """
    
    # 1. Calculate Joint 1 (Shoulder Pan)
    # The arm reaches from a shoulder offset by d[3]
    radius_xy = math.sqrt(x**2 + y**2)
    if radius_xy < d[3]: 
        print("Target too close to base singularity")
        return None

    # Angle to the point
    angle_to_target = math.atan2(y, x)
    
    # Offset angle due to shoulder width (d4)
    # We subtract this for 'Left-Handed' / Standard configuration
    angle_offset = math.acos(d[3] / radius_xy)
    
    theta1 = angle_to_target + angle_offset + (math.pi / 2)
    
    # 2. Calculate Joint 5 (Wrist 1) & Joint 6 (Wrist 2)
    # For a simple top-down grasp (gripper pointing down -Z), 
    # Wrist 1 usually aligns to keep the gripper vertical.
    # This implies P_wrist_center is at Z + d6.
    
    # Wrist Center Position (P_wc)
    # If Tool is pointing DOWN (roll=pi, pitch=0), Z axis is -Z_world
    # We move UP by d[5] (wrist 3 length) to find Wrist Center
    wc_x = x 
    wc_y = y
    wc_z = z + d[5]

    # Project Wrist Center into the shoulder plane (rotate by -theta1)
    # x_s = Coordinate along the arm plane
    # y_s = Z coordinate (up)
    x_s = math.sqrt(wc_x**2 + wc_y**2 - d[3]**2)
    y_s = wc_z - d[0]

    # 3. Calculate Joint 3 (Elbow) - Law of Cosines
    dist_shoulder_to_wc = math.sqrt(x_s**2 + y_s**2)
    
    if dist_shoulder_to_wc > (abs(a[1]) + abs(a[2])):
        print("Target out of reach")
        return None

    cos_theta3 = (dist_shoulder_to_wc**2 - a[1]**2 - a[2]**2) / (2 * a[1] * a[2])
    theta3 = math.acos(cos_theta3) # Positive = Elbow UP

    # 4. Calculate Joint 2 (Shoulder Lift)
    # Angle to wrist center
    alpha = math.atan2(y_s, x_s)
    # Angle of triangle
    beta = math.asin((abs(a[2]) * math.sin(math.pi - theta3)) / dist_shoulder_to_wc)
    
    theta2 = -(alpha + beta) # Negative for UR 'Up' configuration

    # 5. Calculate Joint 4 (Wrist 1)
    # For vertical gripper: theta2 + theta3 + theta4 = -90 degrees (-pi/2)
    theta4 = -(math.pi / 2) - theta2 - theta3

    # 6. Joints 5 & 6 (Hand orientation)
    # For top down:
    theta5 = -math.pi / 2 
    theta6 = 0.0 # Adjust to rotate gripper around Z

    return [theta1, theta2, theta3, theta4, theta5, theta6]


class SimplePickPlace(Node):
    def __init__(self):
        super().__init__('simple_pick_place')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    def move_to(self, x, y, z, duration=4.0):
        # Apply Gripper Offset (approx 16cm from Flange to Tip)
        # We calculate IK for the FLANGE, so we move the target UP by the offset
        flange_z = z + 0.16 
        
        self.get_logger().info(f"Going to: ({x:.2f}, {y:.2f}, {z:.2f}) [Flange Z: {flange_z:.2f}]")
        
        joints = inverse_kinematics_ur5e(x, y, flange_z, 3.14, 0, 0)
        
        if joints:
            self.send_traj(joints, duration)
        else:
            self.get_logger().error("IK Solution Failed")

    def send_traj(self, joints, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        goal.trajectory.points = [point]
        self.arm_client.send_goal_async(goal)
        time.sleep(duration + 0.5)

    def gripper(self, pos):
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = 100.0
        self.gripper_client.send_goal_async(goal)
        time.sleep(1.5)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePickPlace()

    # --- COORDINATES ---
    cube_x = -0.5
    cube_y = 0.01
    cube_z = 0.01 # Center of 5cm cube

    try:
        # 1. Reset
        node.gripper(0.0)
        
        # 2. Pre-Grasp (Hover)
        node.move_to(cube_x, cube_y, cube_z + 0.15)

        # 3. Grasp
        node.move_to(cube_x, cube_y, cube_z)

        # 4. Close
        node.gripper(0.8)

        # 5. Lift
        node.move_to(cube_x, cube_y, cube_z + 0.2)

        # 6. Drop (at Y=0.5)
        node.move_to(0.0, 0.5, 0.2)

        # 7. Release
        node.gripper(0.0)

        # 8. Home
        node.move_to(0.3, 0.0, 0.3)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()