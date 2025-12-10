#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
import math
import time

# ros2 stuff
import tf2_ros
###
# Depricated ROS1 package; using transforms3d or scipy.spatial.transform for ROS2
# from tf2.transformations import quaternion_from_euler, quaternion_matrix 
# from transforms3d import quaternion_matrix, quaternion_from_euler
from scipy.spatial.transform import Rotation as R 
###

from geometry_msgs.msg import Pose, PoseStamped, Vector3, TwistStamped
from std_msgs.msg import Float64MultiArray, Float64, Float32
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

try:
    from xarm.wrapper import XArmAPI
except ImportError: 
    print("xarm package not installed, skipping xarm import")

import jparse_cls


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.get_logger().info("node started")

        # Initialize parameters
        self.base_link = self.declare_parameter('/base_link', 'link_base').get_parameter_value().string_value # defaults are for xarm
        self.end_link = self.declare_parameter('/end_link', 'link_eef').get_parameter_value().string_value

        self.is_sim = self.declare_parameter('~is_sim', False).get_parameter_value().bool_value #boolean to control if the robot is in simulation or not

        self.phi_gain = self.declare_parameter('~phi_gain', 10).get_parameter_value().double_value #gain for the phi term in the JParse method
        self.phi_gain_position = self.declare_parameter('~phi_gain_position', 15).get_parameter_value().double_value #gain for the phi term in the JParse method for position control
        self.phi_gain_angular = self.declare_parameter('~phi_gain_angular', 15).get_parameter_value().double_value #gain for the phi term in the JParse method for angular control

        self.jparse_gamma = self.declare_parameter('~jparse_gamma', 0.1).get_parameter_value().double_value #gamma for the JParse method

        # used to be self.use_space_mouse and self.use_space_mouse_jparse
        self.use_teleop_control = self.declare_parameter('~use_teleop_control', True).get_parameter_value().bool_value #boolean to control if the robot is in teleoperation mode or not
        self.use_teleop_control_jparse = self.declare_parameter('~use_teleop_control_jparse', True).get_parameter_value().bool_value #boolean to control if the robot is in teleoperation mode with JParse
        self.get_logger().info(f"VALUES HERE {self.use_teleop_control} and {self.use_teleop_control_jparse}")
        # used to be self.space_mouse_command
        self.teleop_control_command = np.array([0,0,0,0,0,0]).T

        if self.is_sim == False:
            # from IPython import embed; embed(banner1="hello in not sim")
            self.robot_ip = self.declare_parameter('~robot_ip', '192.168.1.199').get_parameter_value().string_value
            self.arm = XArmAPI(self.robot_ip)
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(state=0)
            time.sleep(1)

            # set joint velocity control mode
            self.arm.set_mode(4)
            self.arm.set_state(0)
            time.sleep(1)

        # set parameters
        self.position_only = self.declare_parameter('~position_only', False).get_parameter_value().bool_value #boolean to control if the robot is in position control mode or not

        # choose the method to use
        self.method = self.declare_parameter('~method', 'JParse').get_parameter_value().string_value #options are JParse, JacobianPseudoInverse, JacobianDampedLeastSquares, JacobianProjection, JacobianDynamicallyConsistentInverse

        self.show_jparse_ellipses = self.declare_parameter('~show_jparse_ellipses', False).get_parameter_value().bool_value #boolean to control if the JParse ellipses are shown or not

        self.joint_states = None
        # self.target_pose = None
        self.teleop_control_command = None
        self.jacobian_calculator = jparse_cls.JParseClass(base_link=self.base_link, end_link=self.end_link)

        #get tf listener
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self)
        self.get_logger().info("ArmController initialized with base link: {}, end link: {}".format(self.base_link, self.end_link))
        self.tf2_timeout = rclpy.duration.Duration(seconds=1.0)

        # gripper initialization
        self.gripper_pose = 900 # means the gripper is open
        self.gripper_speed = 300  # max
        self.gripper_update_rate = 10 # Hz
        self.gripper_min = 0  # closed
        self.gripper_max = 900 # fully opened
        # this enables the gripper and then sets the gripper to open position
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_position(self.gripper_pose)

        
        # Initialize publishers and subscribers
        self.initialize_subscribers()
        self.initialize_publishers()

        # Define the joint names
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

        # Initialize the current positions
        self.current_positions = [0.0] * len(self.joint_names)
        
        self.rate = self.create_rate(10)  # 10 Hz

        self.timer = self.create_timer(0.01, self.timer_callback)


    def timer_callback(self):
        self.control_loop()


    #########################################################################################################
    ############################# SUBSCRIBERS AND PUBLISHERS INITIALIZATION #################################
    #########################################################################################################
    def initialize_subscribers(self):
        self.get_logger().info("Init subs")
        if self.is_sim == True:
            #if we are in simulation, use the joint_states and target_pose topics
            joint_states_topic = '/xarm/joint_states'
        else:
            joint_states_topic = '/joint_states'

        # subscribers
        # js_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        js_cb_g = ReentrantCallbackGroup()
        self.joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_states_callback, 10, callback_group=js_cb_g)
        self.teleop_control_sub = self.create_subscription(
            TwistStamped, 'robot_action', self.teleop_control_callback, 1, callback_group=js_cb_g)
        self.joint_vel_control_sub = self.create_subscription(
            JointTrajectoryPoint, 'joint_vel_control', self.joint_vel_control_callback, 1, callback_group=js_cb_g)
        self.gripper_action_sub = self.create_subscription(
            Float32, '/gripper_action', self.gripper_position_callback, 1, callback_group=js_cb_g)
        
    def initialize_publishers(self):
        # publishers
        self.manip_measure_pub = self.create_publisher(Float64, '/manip_measure', 10)
        self.inverse_cond_number = self.create_publisher(Float64, '/inverse_cond_number', 10)
        #have certain messages to store raw error
        self.pose_error_pub = self.create_publisher(PoseStamped, '/pose_error', 10)
        self.position_error_pub = self.create_publisher(Vector3, '/position_error', 10)
        self.orientation_error_pub = self.create_publisher(Vector3, '/orientation_error', 10)
        #have certain messages to store control error
        self.pose_error_control_pub = self.create_publisher(PoseStamped, '/pose_error_control', 10)
        self.position_error_control_pub = self.create_publisher(Vector3, '/position_error_control', 10)
        self.orientation_error_control_pub = self.create_publisher(Vector3, '/orientation_error_control', 10)
        #publish the current end effector pose and target pose
        self.current_end_effector_pose_pub = self.create_publisher(PoseStamped, '/current_end_effector_pose', 10)
        self.current_target_pose_pub = self.create_publisher(PoseStamped, '/current_target_pose', 10)

        joint_command_topic = self.declare_parameter('~joint_command_topic', '/xarm/xarm7_velo_traj_controller/command').get_parameter_value().string_value
        self.joint_vel_pub = self.create_publisher(JointTrajectory, joint_command_topic, 10)


    ##########################################################################################################
    ############################# CALLBACKS FOR SUBSCRIBERS ##################################################
    ##########################################################################################################
    def gripper_position_callback(self, msg):
        """Callback for the gripper. It includes the gripper position value. Float32 value."""
        joystick_gripper_pose = msg.data
        # convert the joystick value to gripper poses [-1, 1] is to [0, 900], where 0 is close and 900 is open
        delta = joystick_gripper_pose * (self.gripper_speed / self.gripper_update_rate)
        self.gripper_pose += delta
        self.gripper_pose = max(min(self.gripper_pose, self.gripper_max), self.gripper_min)

    def joint_states_callback(self, msg):
        """
        Callback function for the joint_states topic. This function will be called whenever a new message is received
        on the joint_states topic. The message is a sensor_msgs/JointState message, which contains the current joint
        and the corresponding joint velocities and efforts. This function will extract the joint positions, velocities, 
        and efforts and return them as lists.
        """
        self.joint_states = msg

    def teleop_control_callback(self, msg: TwistStamped):
        """
        Callback function for the teleop_control topic (if using an interface). This function will be called whenever a new message is received
        on the teleop_control topic. The message is a geometry_msgs/TwistStamped message, which contains the current
        velocity of the end effector. This function will extract the linear and angular velocities and return them as
        lists.
        """
        teleop_control_command = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]) 
        position_velocity = teleop_control_command[:3]
        angular_velocity = teleop_control_command[3:]
        position_velocity_norm = np.linalg.norm(position_velocity)
        angular_velocity_norm = np.linalg.norm(angular_velocity)
        if position_velocity_norm > 0.05:
            position_velocity = position_velocity / position_velocity_norm * 0.05
        self.teleop_control_command = np.array([position_velocity[0], position_velocity[1],position_velocity[2],angular_velocity[0],angular_velocity[1],angular_velocity[2]])        #check if norm of the space mouse command is greater than 0.05, if so normalize it to this value

    def joint_vel_control_callback(self, msg: JointTrajectoryPoint):
        # Convert JointTrajectory into a list of list of joint velocities
        joint_vels: list[float] = msg.velocities
        self.command_joint_velocities(joint_vels)


    ##########################################################################################################
    ####################################### HELPER FUNCTIONS #################################################
    ##########################################################################################################
    def rad2deg(self, q):
        return q/math.pi*180.0
    
    def EndEffectorPose(self, q):
        """
        This function computes the end-effector pose given the joint configuration q.
        """
        current_pose = PoseStamped()
        current_pose.header.frame_id = self.base_link
        current_pose.header.stamp = self.get_clock().now().to_msg()
        try:
            trans = self.tf2_buffer.lookup_transform(self.base_link, self.end_link, rclpy.time.Time(), timeout=self.tf2_timeout)
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.position.z = trans.transform.translation.z
            current_pose.pose.orientation.x = trans.transform.rotation.x
            current_pose.pose.orientation.y = trans.transform.rotation.y
            current_pose.pose.orientation.z = trans.transform.rotation.z
            current_pose.pose.orientation.w = trans.transform.rotation.w
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error("TF lookup failed")
            self.get_logger().error(f"base link {self.base_link}")
            self.get_logger().error(f"end link {self.end_link}")
        self.current_end_effector_pose_pub.publish(current_pose)
        return current_pose
    
    def rotation_matrix_to_axis_angle(self,R):
        """
        Converts a rotation matrix to an axis-angle vector.
        
        Parameters:
            R (numpy.ndarray): A 3x3 rotation matrix.
        
        Returns:
            numpy.ndarray: Axis-angle vector (3 elements).
        """
        if not (R.shape == (3, 3) and np.allclose(np.dot(R.T, R), np.eye(3)) and np.isclose(np.linalg.det(R), 1)):
            raise ValueError("Input matrix must be a valid rotation matrix.")
        
        # Calculate the angle of rotation
        angle = np.arccos((np.trace(R) - 1) / 2)

        if np.isclose(angle, 0):  # No rotation
            return np.zeros(3)

        if np.isclose(angle, np.pi):  # 180 degrees rotation
            # Special case for 180-degree rotation
            # Extract the axis from the diagonal of R
            axis = np.sqrt((np.diag(R) + 1) / 2)
            # Adjust signs based on the matrix off-diagonals
            axis[0] *= np.sign(R[2, 1] - R[1, 2])
            axis[1] *= np.sign(R[0, 2] - R[2, 0])
            axis[2] *= np.sign(R[1, 0] - R[0, 1])
            return axis * angle

        # General case
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * np.sin(angle))

        return axis * angle

    def pose_error(self, current_pose, target_pose, error_norm_max = 0.05, control_pose_error = True):
        """
        This function computes the error between the current pose and the target pose.
        NOT NEEDED
        """
        self.get_logger().info(f'TARGET POSE: {target_pose}')
        #Compute the position error
        position_error = np.array([target_pose.pose.position.x - current_pose.pose.position.x,
                                   target_pose.pose.position.y - current_pose.pose.position.y,
                                   target_pose.pose.position.z - current_pose.pose.position.z])
        #if the norm of the position error is greater than the maximum allowed, scale it down
        if control_pose_error == True:
            if np.linalg.norm(position_error) > error_norm_max:
                position_error = position_error / np.linalg.norm(position_error) * error_norm_max

        # for ros 2
        goal_quat_scripy = np.array([target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z])
        current_rotation_quat_scipy = np.array([current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z])
        goal_rotation = R.from_quat(goal_quat_scripy)
        current_rotation = R.from_quat(current_rotation_quat_scipy)

        goal_rotation_matrix = goal_rotation.as_matrix()
        current_rotation_matrix = current_rotation.as_matrix()

        #Compute the orientation error
        R_error= np.dot(goal_rotation_matrix, np.linalg.inv(current_rotation_matrix))
        # Extract the axis-angle lie algebra vector from the rotation matrix
        angle_error = self.rotation_matrix_to_axis_angle(R_error)
        # Return the position and orientation error
        return position_error, angle_error

    def axis_angle_to_rotation_matrix(self, axis_angle):
        """
        Converts an axis-angle vector to a rotation matrix.
        
        Parameters:
            axis_angle (numpy.ndarray): Axis-angle vector (3 elements).
        
        Returns:
            numpy.ndarray: A 3x3 rotation matrix.
        """
        # Extract the axis and angle
        axis = axis_angle / np.linalg.norm(axis_angle)
        angle = np.linalg.norm(axis_angle)
        
        # Compute the skew-symmetric matrix
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        
        # Compute the rotation matrix using the Rodrigues' formula
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
        
        return R
    
    def rotation_matrix_to_quaternion(self, R):
        """
        Converts a rotation matrix to a quaternion.
        
        Parameters:
            R (numpy.ndarray): A 3x3 rotation matrix.
        
        Returns:
            numpy.ndarray: A 4-element quaternion.
        """
        if not (R.shape == (3, 3) and np.allclose(np.dot(R.T, R), np.eye(3)) and np.isclose(np.linalg.det(R), 1)):
            raise ValueError("Input matrix must be a valid rotation matrix.")
        
        # Compute the quaternion using the method from the book
        q = np.zeros(4)
        q[0] = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        q[1] = (R[2, 1] - R[1, 2]) / (4 * q[0])
        q[2] = (R[0, 2] - R[2, 0]) / (4 * q[0])
        q[3] = (R[1, 0] - R[0, 1]) / (4 * q[0])
        
        return q
    
    def velocity_home_robot(self):
        """
        This function commands the robot to the home position using joint velocities.
        """
        # do the following in a loop for 5 seconds:
        if self.joint_states is not None:
            q = []
            dq = []
            for joint_name in self.joint_names:
                idx = self.joint_states.name.index(joint_name)
                q.append(self.joint_states.position[idx])
                dq.append(self.joint_states.velocity[idx])
            self.current_positions = q
            # Command the joint velocities
            if self.is_sim == True:
                kp_gain = 10.0
            else:
                kp_gain = 0.5
            q_home = [-0.03142359480261803, -0.5166178941726685, 0.12042707949876785, 0.9197863936424255, -0.03142168000340462, 1.4172008037567139, 0.03490765020251274]
            #now find the error between the current position and the home position and use joint velocities to move towards the home position
            joint_velocities_list = kp_gain * (np.array(q_home) - np.array(q))
            self.get_logger().info(f"joints for homing {joint_velocities_list}")
            self.command_joint_velocities(joint_velocities_list)

    
    ##############################################################################################################
    ######################################### COMMANDS TO THE ROBOT ##############################################
    ##############################################################################################################
    def publish_pose_errors(self, position_error, angular_error, control_pose_error = True):
        """
        Publishes the position and orientation errors as ROS messages.
        """
        pose_error_msg = PoseStamped()
        pose_error_msg.header.frame_id = self.base_link
        pose_error_msg.header.stamp = self.get_clock().now().to_msg()
        pose_error_msg.pose.position.x = position_error[0]
        pose_error_msg.pose.position.y = position_error[1]
        pose_error_msg.pose.position.z = position_error[2]
        #for completeness, convert the axis angle to quaternion
        Rmat = self.axis_angle_to_rotation_matrix(angular_error)
        quat_from_Rmat = self.rotation_matrix_to_quaternion(Rmat)
        # quat_from_Rmat = tf.transformations.quaternion_from_matrix(Rmat)
        pose_error_msg.pose.orientation.x = quat_from_Rmat[0]
        pose_error_msg.pose.orientation.y = quat_from_Rmat[1]
        pose_error_msg.pose.orientation.z = quat_from_Rmat[2]
        pose_error_msg.pose.orientation.w = quat_from_Rmat[3]
        #publish the axis-angle orientation error
        orientation_error_msg = Vector3()
        orientation_error_msg.x = angular_error[0]
        orientation_error_msg.y = angular_error[1]
        orientation_error_msg.z = angular_error[2]
        #publish the position error
        position_error_msg = Vector3()
        position_error_msg.x = position_error[0]
        position_error_msg.y = position_error[1]
        position_error_msg.z = position_error[2]

        #Now publish
        if control_pose_error == True:
            self.position_error_control_pub.publish(position_error_msg)
            self.pose_error_control_pub.publish(pose_error_msg)
            self.orientation_error_control_pub.publish(orientation_error_msg)
        else:
            self.pose_error_pub.publish(pose_error_msg)
            self.position_error_pub.publish(position_error_msg)
            self.orientation_error_pub.publish(orientation_error_msg)

    def command_joint_velocities(self, joint_vel_list: list[float]):
        """
        This function commands the joint velocities to the robot using the appropriate ROS message type.
        """
        if self.is_sim:
            # Create the JointTrajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = self.joint_names

            # Create a trajectory point
            point = JointTrajectoryPoint()

            # Use current positions
            point.positions = self.current_positions

            # Set velocities
            #make velocity negative because Xarm has cw positive direction for joint velocities
            joint_vel_list = [-v for v in joint_vel_list]
            point.velocities = joint_vel_list #this is negative because Xarm has cw positive direction for joint velocities

            # Set accelerations to zero
            point.accelerations = [0.0] * len(self.joint_names)

            # Effort (optional; set to None or skip if not needed)
            point.effort = [0.0] * len(self.joint_names)

            # Set the time from start
            point.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg()  # 100 ms

            # Add the point to the trajectory
            trajectory_msg.points.append(point)

            # Publish the trajectory
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.joint_vel_pub.publish(trajectory_msg)
        else:
            # this is on the real robot, directly send joint velocities
            # Send joint velocities to the arm
            # log the velocities
            if self.use_teleop_control_jparse:
                # self.get_logger().info(f"Joint velocities: {joint_vel_list}")
                self.arm.vc_set_joint_velocity(joint_vel_list, is_radian=True) # arm command
                self.arm.set_gripper_position(self.gripper_pose) # gripper command

    ###############################################################################################################
    ######################################### CONTROL LOOP ########################################################
    ###############################################################################################################
    def control_loop(self):
        """
        This function implements the control loop for the arm controller.

        TODO: Need to compute the jacobian and then pass that to JParse and dont need all the comparisons!! (8/8/25)
        """

        # while rclpy.ok():

        if self.joint_states is not None and self.teleop_control_command is not None:  # edited this 7/11/25
            t = self.get_clock().now()
            # obtain the current joints
            q = []
            dq = []
            effort = []
            for joint_name in self.joint_names:
                idx = self.joint_states.name.index(joint_name)
                q.append(self.joint_states.position[idx])
                dq.append(self.joint_states.velocity[idx])
                effort.append(self.joint_states.effort[idx])  
            self.current_positions = q
            # Calculate the JParsed Jacobian
            # NEW # 8/8/2025
            # TODO(Sharwin or Demiana): Use pinocchio function to access example instead of public repo
            urdf_filename = "/home/workspace/src/example-robot-data/robots/xarm_description/urdf/xarm7.urdf"
            model = pin.buildModelFromUrdf(urdf_filename) # the model structure of the robot
            data = model.createData() # the data structure of the robot 
            # 1) Forward kinematics
            # from IPython import embed; embed(banner1="computing jacobians")
            np_q = np.array(q)
            if isinstance(np_q, np.ndarray):
                print("The object is a NumPy array.")
                self.get_logger().info(f'q: {np_q} {np_q}')
            else:
                print("The object is not a NumPy array.")
            
            pin.forwardKinematics(model, data, np_q)  # needs to take in a numpy array
            pin.updateFramePlacements(model, data)

            # 2) Compute joint Jacobians
            pin.computeJointJacobians(model, data, np_q)

            # 3) Extract Jacobian
            J6 = data.J #J3 = data.J[:3, :] -- will give us only position so removing it
            print(f'JACOBIAN: {J6}')
            
            method = self.method #set by parameter, can be set from launch file
            self.get_logger().info(f"Method being used: {method}")
            if method == "JacobianPseudoInverse":
                raise NotImplementedError
            elif method == "JParse":
                J_method, J_nullspace = self.jacobian_calculator.JParse(J=J6, jac_nullspace_bool=True, gamma=0.1, singular_direction_gain_position=1, singular_direction_gain_orientation=2, position_dimensions=3, angular_dimensions=3)
            elif method == "JacobianDampedLeastSquares":
                raise NotImplementedError
            elif method == "JacobianProjection":
                raise NotImplementedError
            elif method == "JacobianSafety":
                raise NotImplementedError
            elif method == "JacobianSafetyProjection":
                raise NotImplementedError
            
            #Calculate the delta_x (task space error)
            current_pose = self.EndEffectorPose(q)
            if self.is_sim == False:
                #real world limits
                error_norm_max = 0.10
            else:
                #simulation limits
                error_norm_max = 1.0         
            #move in nullspace towards nominal pose (for xarm its 0 joint angle for Xarm7)
            if self.is_sim == True:
                kp_gain = 2.0
            else:
                # kp_gain = 1.0
                kp_gain = 3.0

            if self.is_sim == True:
                nominal_motion_nullspace = np.matrix([-v*kp_gain for v in q]).T #send to home which is 0 joint position for all joints
            else:
                # nominal motion nullspace which checks if q magnitude is below threshold and chooses the minimum
                null_space_angle_rate = 0.6

                nominal_motion_nullspace = np.matrix([np.sign(-v*kp_gain)*np.min([np.abs(-v*kp_gain),null_space_angle_rate]) for v in q]).T #send to home which is 0 joint position for all joints

            # Calculate and command the joint velocities
            if self.position_only == True:
                self.get_logger().info("Position only control")
                # position_error = np.matrix(position_error).T
                raise NotImplementedError
            else:
                # realworld gains (tested)
                kp_gain_position = 1.0
                kp_gain_orientation = 1.0
                if self.is_sim == True:
                    #use these gains only in simulation!
                    kp_gain_position = 10.0
                    kp_gain_orientation = 10.0
                Kp_position = np.diag([kp_gain_position, kp_gain_position, kp_gain_position])  # Proportional gain matrix
                Kp_orientation = np.diag([kp_gain_orientation, kp_gain_orientation, kp_gain_orientation])
                if self.use_teleop_control == False:
                    # joint_velocities = J_method @ full_pose_error + J_nullspace @ nominal_motion_nullspace
                    raise NotImplementedError
                if self.use_teleop_control == True:
                    #use the teleop control command to control the joint velocities
                    teleop_control_command = np.matrix(self.teleop_control_command).T
                    #now add this to the joint velocities
                    joint_velocities = J_method @ teleop_control_command + J_nullspace @ nominal_motion_nullspace
            joint_velocities_list = np.array(joint_velocities).flatten().tolist()
            # command the joint velocities
            self.command_joint_velocities(joint_velocities_list) #this commands the joint velocities
            self.get_logger().info("Control loop running")

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()