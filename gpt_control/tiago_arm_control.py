#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes
from pymoveit2 import MoveIt2
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose, PoseStamped
from linkattacher_msgs.srv import AttachLink, DetachLink

import math
import time

class TiagoTableApproach(Node):
    def __init__(self):
        super().__init__('tiago_table_approach')
        
        # Create service client for getting entity state
        self.get_entity_client = self.create_client(
            GetEntityState, 
            '/gazebo/get_entity_state'
        )
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create publisher for gripper control
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        # Initialize MoveIt2 for arm control
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
            ],
            base_link_name="base_footprint",
            end_effector_name="gripper_grasping_frame",
            group_name="arm"
        )
        
        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for service to be available
        while not self.get_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/get_entity_state service...')
        
        self.get_logger().info('Service available!')


    def get_ee_pose(self):
    # FK for current joint state and default ee link
        ee = self.moveit2.compute_fk()
        if isinstance(ee, PoseStamped):
            ee_pose = ee.pose
        else:
            # Already a Pose
            ee_pose = ee

        self.get_logger().info(
            f"EE pose: x={ee_pose.position.x:.3f}, "
            f"y={ee_pose.position.y:.3f}, "
            f"z={ee_pose.position.z:.3f}"
        )
        return ee_pose

    def get_table_position(self):
        """Get the table position from Gazebo"""
        request = GetEntityState.Request()
        request.name = 'table'
        request.reference_frame = 'world'
        
        future = self.get_entity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                pos = response.state.pose.position
                self.get_logger().info(f'Table position: x={pos.x}, y={pos.y}, z={pos.z}')
                return pos.x, pos.y
            else:
                self.get_logger().error('Failed to get table state')
                return None, None
        else:
            self.get_logger().error('Service call failed')
            return None, None

    def move_towards_table(self, table_x, table_y, distance_from_table=1.0):
        """Move the robot towards the table, stopping at a certain distance"""
        robot_x, robot_y = 0.0, 0.0
        
        # Calculate direction vector to table
        dx = table_x - robot_x
        dy = table_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate target position (stop at distance_from_table meters from table)
        target_distance = distance - distance_from_table
        
        if target_distance <= 0.5:
            self.get_logger().info('Already close enough to table')
            return
        
        # Calculate angle to table
        target_angle = math.atan2(dy, dx)
        
        self.get_logger().info(f'Moving towards table: distance={distance:.2f}m, angle={math.degrees(target_angle):.2f}°')
        
        # First rotate to face the table
        twist = Twist()
        twist.angular.z = 0.3 if target_angle > 0 else -0.3
        
        rotation_time = abs(target_angle) / 0.3
        self.get_logger().info(f'Rotating for {rotation_time:.2f} seconds')
        
        start_time = time.time()
        while (time.time() - start_time) < rotation_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        
        # Move forward towards table
        twist.linear.x = 0.2  # 0.2 m/s forward
        movement_time = target_distance / 0.2
        
        self.get_logger().info(f'Moving forward for {movement_time:.2f} seconds')
        
        start_time = time.time()
        while (time.time() - start_time) < movement_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('Reached target position in front of table')

    
    def get_object_position(self, object_name):
        """Get the object position from Gazebo"""
        request = GetEntityState.Request()
        request.name = object_name
        request.reference_frame = 'world'
        
        future = self.get_entity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                pos = response.state.pose.position
                self.get_logger().info(f'{object_name} position: x={pos.x}, y={pos.y}, z={pos.z}')
                return pos.x, pos.y, pos.z
            else:
                self.get_logger().error(f'Failed to get {object_name} state')
                return None, None, None
        else:
            self.get_logger().error('Service call failed')
            return None, None, None
        
    def get_object_position_in_base_frame(self, object_name):
        """Get object position in base_footprint frame via TF."""
        # 1) Get object pose in world frame from Gazebo
        obj_x, obj_y, obj_z = self.get_object_position(object_name)
        if obj_x is None:
            return None, None, None

        obj_world = PoseStamped()
        obj_world.header.frame_id = "world"
        obj_world.pose.position.x = obj_x
        obj_world.pose.position.y = obj_y
        obj_world.pose.position.z = obj_z
        obj_world.pose.orientation.w = 1.0  # no rotation; we only care about position

        try:
            # 2) Transform world → base_footprint
            transform = self.tf_buffer.lookup_transform(
                "base_footprint",   # target frame
                "world",            # source frame
                rclpy.time.Time()
            )
            obj_base = do_transform_pose(obj_world, transform)

            x = obj_base.pose.position.x
            y = obj_base.pose.position.y
            z = obj_base.pose.position.z

            self.get_logger().info(
                f"{object_name} in base_footprint: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )
            return x, y, z

        except TransformException as ex:
            self.get_logger().error(f"TF error world→base_footprint: {ex}")
            return None, None, None



    def move_infront_object(self, robot_x, robot_y, object_x, object_y, distance_from_object=0.5):
        """Move the robot in front of an object without rotating - only x and y movement"""

        # Calculate direction to object
        dx = object_x - robot_x
        dy = object_y - robot_y
        
        self.get_logger().info(f'Moving to object: dx={dx:.2f}m, dy={dy:.2f}m')
        
        twist = Twist()
        
        # First move forward (along x-axis)
        if abs(dx) > 0.05:  # If there's significant x distance
            move_x = dx - distance_from_object if dx > 0 else dx + distance_from_object
            if abs(move_x) > 0.05:
                twist.linear.x = 0.2 if move_x > 0 else -0.2
                movement_time = abs(move_x) / 0.2
                
                self.get_logger().info(f'Moving forward/backward: {move_x:.2f}m for {movement_time:.2f}s')
                
                start_time = time.time()
                while (time.time() - start_time) < movement_time:
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.1)
                
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.3)
        
        # Then shift left/right (along y-axis)
        if abs(dy) > 0:  # If there's significant y distance
            twist.linear.y = 0.2 if dy > 0 else -0.2
            movement_time = abs(dy) / 0.2
            
            self.get_logger().info(f'Shifting left/right: {dy:.2f}m for {movement_time:.2f}s')
            
            start_time = time.time()
            while (time.time() - start_time) < movement_time:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            twist.linear.y = 0.0
            self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('Reached position in front of object')


    def attach_object(self, model1='tiago', link1='arm_6_link',  model2='cocacola', link2='link'):
        req = AttachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2

        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp and resp.success:  # field name depends on the .srv, check once
            self.get_logger().info("Attached object to gripper.")
            return True
        else:
            self.get_logger().error("Attach failed.")
            return False



    def grab_object(self, object_x, object_y, object_z):
        """Grab an object using MoveIt2 with Cartesian coordinates"""
        self.get_logger().info(f'Grabbing object at position x={object_x}, y={object_y}, z={object_z}')
        
        # Step 1: Open gripper
        self.get_logger().info('Opening gripper...')
        gripper_traj = JointTrajectory()
        gripper_traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        point.time_from_start.sec = 1
        gripper_traj.points = [point]
        
        self.gripper_pub.publish(gripper_traj)
        time.sleep(2.0)
        
        # Step 2: Move arm to object position
        self.get_logger().info(f'Moving arm to object...')
        
        # Adjust z coordinate (gripper approach from above)
        approach_z = object_z + 0.1  # 10cm above object
        

        """
        # Move to approach position first
        self.moveit2.move_to_pose(
            position=[object_x, object_y, approach_z],
            quat_xyzw=[0.0, 0.707, 0.0, 0.707],  # Gripper pointing down
            cartesian=True
        )
        self.moveit2.wait_until_executed()
        
        self.get_logger().info('Reached approach position, moving down to grasp...')
        time.sleep(1.0)
        
        # Move down to grasp position
        self.moveit2.move_to_pose(
            position=[object_x, object_y, object_z - 0.02],  # Slightly lower to grasp
            quat_xyzw=[0.0, 0.707, 0.0, 0.707],
            cartesian=True
        )
        self.moveit2.wait_until_executed()
        
        time.sleep(1.0)
        """
        
        # Step 3: Close gripper
        self.get_logger().info('Closing gripper...')
        point.positions = [0.0, 0.0]
        point.time_from_start.sec = 1
        gripper_traj.points = [point]
        
        self.gripper_pub.publish(gripper_traj)
        time.sleep(2.0)
        
        self.get_logger().info('Object grabbed!')


    def move_gripper(self, position_x, position_y, position_z, orientation_xyzw=[0.0075238, -0.027324, -0.0015946, 0.9997]):
        """Move the gripper to a specified position using MoveIt2"""
        self.get_logger().info(f'Moving gripper to position x={position_x}, y={position_y}, z={position_z}')
        
        self.moveit2.move_to_pose(
            position=[position_x, position_y, position_z],
            quat_xyzw=orientation_xyzw,
            cartesian=True
        )
        self.moveit2.wait_until_executed()
        
        self.get_logger().info('Gripper moved to specified position')

    def get_gripper_position(self):
        """Get gripper position relative to base_footprint"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint',
                'gripper_grasping_frame',
                rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            self.get_logger().info(f'Gripper position: x={x:.3f}, y={y:.3f}, z={z:.3f}')
            return x, y, z
        except TransformException as ex:
            self.get_logger().error(f'Could not get gripper transform: {ex}')
            return None, None, None

    def get_object_position_in_base_frame(self, object_name):
        """Get object position relative to base_footprint using world position and robot position"""
        # Get object position in world frame
        obj_x, obj_y, obj_z = self.get_object_position(object_name)
        if obj_x is None:
            return None, None, None
        
        # Get robot position in world frame
        robot_x, robot_y, robot_z = self.get_object_position('tiago')
        if robot_x is None:
            return None, None, None
        
        # Calculate relative position
        rel_x = obj_x - robot_x
        rel_y = obj_y - robot_y
        rel_z = obj_z
        
        self.get_logger().info(f'{object_name} relative position: x={rel_x:.3f}, y={rel_y:.3f}, z={rel_z:.3f}')
        return rel_x, rel_y, rel_z



    def grasp_object_by_name(self, object_name, lift_above=0.10, grasp_offset=-0.02):
        """
        Grasp an object using object + gripper poses in base_footprint.
        lift_above: how much above the object to go for pre-grasp (m)
        grasp_offset: how much below the object z for final grasp (m)
        """
        # 1) Object pose in base_footprint
        obj_x, obj_y, obj_z = self.get_object_position_in_base_frame(object_name)
        if obj_x is None:
            self.get_logger().error("Cannot get object pose, aborting grasp.")
            return False

        # 2) Current gripper pose (for logging / potential smarter strategy)
        ee_pose = self.get_ee_pose()
        if ee_pose is None:
            self.get_logger().error("Cannot get EE pose, aborting grasp.")
            return False

        self.get_logger().info(
            f"EE at x={ee_pose.position.x:.3f}, y={ee_pose.position.y:.3f}, z={ee_pose.position.z:.3f}"
        )
        self.get_logger().info(
            f"Object {object_name} at x={obj_x:.3f}, y={obj_y:.3f}, z={obj_z:.3f} in base frame"
        )

        # 3) Open gripper
        self.get_logger().info("Opening gripper...")
        gripper_traj = JointTrajectory()
        gripper_traj.joint_names = [
            "gripper_left_finger_joint",
            "gripper_right_finger_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        point.time_from_start.sec = 1
        gripper_traj.points = [point]
        self.gripper_pub.publish(gripper_traj)
        time.sleep(2.0)


        self.get_logger().info("Moving to pre-grasp pose above object...")
        self.move_gripper(
            position_x=obj_x-0.3,
            position_y=obj_y,
            position_z=obj_z,
            orientation_xyzw=[0.0075238, -0.027324, -0.0015946, 0.9996],  # pointing to the side of the object
        )

        self.attach_object(model2=object_name, link2='link')

        # 5) Close gripper
        #self.get_logger().info("Closing gripper...")
        #point.positions = [0.0, 0.0]
        #point.time_from_start.sec = 1
        #gripper_traj.points = [point]
        #self.gripper_pub.publish(gripper_traj)
        #time.sleep(2.0)

        self.get_logger().info("Object grasp attempt finished.")
        return True



    def run(self):
        """Main execution method"""
        # Wait for TF to be ready
        time.sleep(2.0)

        table_x, table_y = self.get_table_position()
        self.move_gripper(0.83578, -0.022147, 0.94412)  # Move gripper to a safe position above the table
        self.move_towards_table(table_x, table_y, distance_from_table=0.8)
        self.grasp_object_by_name('cocacola', lift_above=0.15, grasp_offset=-0.02)
        
        # Get current gripper position
        #self.get_gripper_position()
        
        # Align gripper with coke (lift 15cm above to avoid collision)
        #self.align_gripper_with_object('cocacola', lift_offset=0.15)

def main(args=None):
    rclpy.init(args=args)
    
    node = TiagoTableApproach()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()