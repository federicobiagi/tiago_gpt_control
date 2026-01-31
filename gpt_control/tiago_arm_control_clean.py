#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymoveit2 import MoveIt2
from linkattacher_msgs.srv import AttachLink, DetachLink


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotation_matrix_z(yaw): #because we only care about the orientation around the z-axis
    """Returns a 3x3 rotation matrix for rotation around the z-axis (yaw)."""
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    return np.array([
        [cos_yaw, -sin_yaw, 0.0],
        [sin_yaw,  cos_yaw, 0.0],
        [0.0,      0.0,     1.0]
    ])


def transform_to_robot_frame(world_position, robot_position, robot_orientation):
    """
    Transform a point from world frame to robot's base_footprint frame.
    
    Args:
        world_position: (x, y, z) position in world frame
        robot_position: (x, y, z) robot position in world frame
        robot_orientation: quaternion (x, y, z, w)
    
    Returns:
        (x_robot, y_robot, z_robot) position in robot's base frame
    """
    # Translation: Get position relative to robot in world frame
    p_world = np.array([world_position.x, world_position.y, world_position.z])
    robot_world = np.array([robot_position.x, robot_position.y, robot_position.z])
    p_relative_world = p_world - robot_world

    # Rotation: Convert from world frame to robot frame
    yaw = yaw_from_quaternion(robot_orientation)
    R_world_to_robot = rotation_matrix_z(-yaw)  # Negative yaw inverts the transformation
    p_robot = R_world_to_robot @ p_relative_world
    
    return p_robot[0], p_robot[1], p_robot[2]


class TiagoControl(Node):
    def __init__(self):
        super().__init__("tiago_control_node")

        # Crea un client per il ros service fornito da gazebo per ottenere lo stato di un'entità nella simulazione
        self.get_entity_client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        # Crea un publisher per il topic /cmd_vel per controllare il movimento della base del robot
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Crea un publisher per il topic del controller del gripper
        self.gripper_pub = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        # Crea i client per i ros service di attacco e distacco dei link per attaccarre oggetti al gripper del robot
        self.attach_client = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_client = self.create_client(DetachLink, "/DETACHLINK")
        # Inizializza MoveIt2 per il controllo del braccio del robot
        self.moveit2 = MoveIt2(node=self, joint_names=["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], base_link_name="base_footprint", end_effector_name="gripper_grasping_frame", group_name="arm")
        #self.moveit2 = MoveIt2(node=self, joint_names=["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], base_link_name="base_footprint", end_effector_name="gripper_link", group_name="arm")
        while not self.get_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /gazebo/get_entity_state...")
        self.get_logger().info("Gazebo entity service available.")

        self.get_logger().info("Tiago control node initialized.")



    def get_entity_position(self, name: str):
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = "world"
        future = self.get_entity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        pose = future.result().state.pose
        return pose

    def get_table_position(self):
        pose = self.get_entity_position("table")
        self.get_logger().info(f"Table @ x={pose.position.x:.3f}, y={pose.position.y:.3f}")
        return pose.position.x, pose.position.y

    def get_object_position_in_robot_frame(self, object_name: str):
        """
        Get object position in robot's base_footprint frame.
        Uses transformation matrices for clarity.
        """
        obj_pose = self.get_entity_position(object_name)
        robot_pose = self.get_entity_position("tiago")

        x_b, y_b, z_b = transform_to_robot_frame(
            obj_pose.position,
            robot_pose.position,
            robot_pose.orientation
        )

        self.get_logger().info(
            f"{object_name} in base_footprint: x={x_b:.3f}, y={y_b:.3f}, z={z_b:.3f}"
        )
        return x_b, y_b, z_b

    def get_ee_pose(self):
        ee = self.moveit2.compute_fk()
        return ee.pose if hasattr(ee, "pose") else ee


    def move_towards_table(self, table_x, table_y, distance_from_table=1.0):
        robot_x, robot_y = 0.0, 0.0
        dx = table_x - robot_x
        dy = table_y - robot_y
        distance = math.sqrt(dx * dx + dy * dy)
        target_distance = distance - distance_from_table
        if target_distance <= 0.5:
            self.get_logger().info("Already close enough to table.")
            return
        angle = math.atan2(dy, dx)
        self.get_logger().info(f"Move to table: dist={distance:.2f} m, angle={math.degrees(angle):.1f}°")
        self.rotate_angle(angle, angular_speed=0.3)
        time.sleep(0.5)
        self.move_distance(distance=target_distance, speed=0.2)
        self.get_logger().info("Base positioned in front of table.")


    def move_distance(self, distance=0.4, speed=0.2):
        """Move the base by `distance` meters using /cmd_vel."""
        twist = Twist()
        direction = 1.0 if distance >= 0.0 else -1.0
        vel = abs(speed) * direction

        twist.linear.x = vel
        duration = abs(distance) / abs(speed)
        start = self.get_clock().now().nanoseconds / 1e9

        while (self.get_clock().now().nanoseconds / 1e9 - start) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def rotate_angle(self, angle, angular_speed=0.3):
        twist = Twist()
        direction = 1.0 if angle >= 0.0 else -1.0
        twist.angular.z = abs(angular_speed) * direction
        duration = abs(angle) / abs(angular_speed)
        start = self.get_clock().now().nanoseconds / 1e9
        while (self.get_clock().now().nanoseconds / 1e9 - start) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


    def open_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [0.044, 0.044]
        pt.time_from_start.sec = 1
        traj.points = [pt]
        self.gripper_pub.publish(traj)
        time.sleep(2.0)

    def close_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [0.022, 0.022]
        pt.time_from_start.sec = 1
        traj.points = [pt]
        self.gripper_pub.publish(traj)
        time.sleep(2.0)


    def move_gripper(self, x, y, z, quat_xyzw=(0.0075238, -0.027324, -0.0015946, 0.9997)):
        self.get_logger().info(f"Move gripper to x={x:.3f}, y={y:.3f}, z={z:.3f}")
        self.moveit2.move_to_pose(position=[x, y, z], quat_xyzw=list(quat_xyzw), cartesian=True)
        self.moveit2.wait_until_executed()


    def move_arm(self, joint_positions):  #in radians
        self.get_logger().info(f"Moving arm to joints: {joint_positions}")
        joint_positions = [float(pos) for pos in joint_positions]
        if len(joint_positions) != 7:
            self.get_logger().error("move_arm expects exactly 7 joint positions")
            return

        # Joint limits
        # arm_1: [0.0, 2.748] (0 to 157.5 deg)
        # arm_2: [-1.570, 1.090] (-90 to 62.5 deg)
        # arm_3: [-3.534, 1.570] (-202.5 to 90 deg)
        # arm_4: [-0.392, 2.356] (-22.5 to 135 deg)
        # arm_5: [-2.094, 2.094] (-120 to 120 deg)
        # arm_6: [-1.570, 1.570] (-90 to 90 deg)
        # arm_7: [-2.094, 2.094] (-120 to 120 deg)
        
        limits = [
            (0.0, 2.748),      # arm_1
            (-1.570, 1.090),   # arm_2
            (-3.534, 1.570),   # arm_3
            (-0.392, 2.356),   # arm_4
            (-2.094, 2.094),   # arm_5
            (-1.570, 1.570),   # arm_6
            (-2.094, 2.094)    # arm_7
        ]

        for i, (pos, (low, high)) in enumerate(zip(joint_positions, limits)):
            if not (low <= pos <= high):
                self.get_logger().error(f"Joint arm_{i+1}_joint value {pos:.3f} is out of limits [{low:.3f}, {high:.3f}]")
                return

        # Set the joint goal
        self.moveit2.move_to_configuration(joint_positions=joint_positions,joint_names=["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"])
        self.moveit2.wait_until_executed()
        self.get_logger().info("Arm movement finished.")


    def attach_object(self, model1="tiago", link1="arm_6_link", model2="cocacola", link2="link"):
        req = AttachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("AttachLink called.")


    def detach_object(self, model1="tiago", link1="arm_6_link", model2="cocacola", link2="link"):
        req = DetachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("DetachLink called.")

    def grasp_object_by_name_front(self, object_name: str, approach_dist: float = 0.30, grasp_offset: float = 0.05, height_offset: float = 0.10):
        obj_x, obj_y, obj_z = self.get_object_position_in_robot_frame(object_name)
        ee_pose = self.get_ee_pose()
        ee_quat = [ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w]
        self.get_logger().info(f"EE @ x={ee_pose.position.x:.3f}, y={ee_pose.position.y:.3f}, z={ee_pose.position.z:.3f}")

        self.open_gripper()

        # Determine approach direction, if the object is in front of the robot or not   
        direction = 1.0 if obj_x > 0.0 else -1.0
        # Position "approach_dist" cm away from the object along x
        pre_x = obj_x - direction * approach_dist

        # Define also the height of grasp
        target_z = obj_z + height_offset

        self.move_gripper(pre_x, obj_y, target_z, ee_quat)

        # Now position to grasp the object, position 5cm away from the object center to avoid compenetration
        grasp_x = obj_x - direction * grasp_offset

        self.move_gripper(grasp_x, obj_y, target_z, ee_quat)
        self.attach_object(model2=object_name, link2="link")

        # Rise the end effector
        self.move_gripper(pre_x, obj_y, target_z + 0.25, ee_quat)
        self.get_logger().info("Front grasp sequence finished.")


    def run(self):
        time.sleep(2.0)
        table_pose = self.get_object_position_in_robot_frame("table")
        self.move_gripper(0.83578, -0.022147, 0.94412)
        self.move_towards_table(table_pose[0], table_pose[1], distance_from_table=0.8)

        self.grasp_object_by_name_front("cocacola")
        self.detach_object(model2="cocacola", link2="link")

        time.sleep(1.0)
        self.move_gripper(0.53578, -0.022147, 0.94412)
        self.move_gripper(0.53578, -0.022147, 0.94412)
        self.rotate_angle(angle=math.radians(90), angular_speed=0.5)
        self.move_distance(distance=0.3, speed=0.2)
        self.rotate_angle(angle=math.radians(-90), angular_speed=0.5)
        self.grasp_object_by_name_front("biscuits_pack")

        self.detach_object(model2="biscuits_pack", link2="link")


        


def main(args=None):
    rclpy.init(args=args)
    node = TiagoControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
