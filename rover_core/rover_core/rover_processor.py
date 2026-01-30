import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import os
import configparser
import serial
from serial.tools import list_ports
from rover_core.kinematics import skid_steering_evaluation, tank_steering_evaluation
# ops course specific
from .cobs import encode, decode, DecodeError
import threading
import struct
import time

from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np

from rover_core.kinematics_analytic import fkne, fkne_3d, ikne, ikne_3d, joint_limits

ENABLE_STATE_TOPIC = "enable_state"
PLAY_MODE_COMMAND_TOPIC = "cmd_vel"
CONFIG_FILE = "robot_config.cfg"


class RoverProcessor(Node):
    def __init__(self):

        #------------VARIABLE INITIALISATION-------------

        # Initialize the class variables
        self.state = "skid"
        self.motor_gain = 1.0
        self.linear_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.angular_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.drive_unit_publishers = []
        # Get config file dir
        config_file_path = os.getcwd()
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE))  
        # Read config file
        config = configparser.ConfigParser()
        if not config.read(config_file_path):
            print(f"Config file {CONFIG_FILE} not found or invalid.")
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return
        # Assign new vars for code readability
        node_name                   = "core"
        robot_name                  = config['Robot']['name']
        control_topic               = config['Robot']['control_topic']
        queue_length                = int(config['Robot']['queue_length'])
        encoder_topic               = config['Roboteq']['encoder_topic']
        encoder_timer_frequency     = float(config['Roboteq']['encoder_timer'])
        sonar_timer_frequency       = float(config['Robot']['sonar_timer'])
        self.motor_gain             = float(config['Movebase_Kinematics']['gain'])
        self.enable_topic_name      = f'{robot_name}/{ENABLE_STATE_TOPIC}'
        self.baudrate               = config['Roboteq']['baudrate']
        self.encoder_cmd            = config['Roboteq']['encoder_command']
        self.devices                = {}
        self.port_ids               = {}
        self.port_ids['Roboteq'] = list(
            map(lambda s: int(s.strip(), 0),
                config['Roboteq']['port_ids'].split(','))
        )
        self.port_ids['RedBoard'] = list(
            map(lambda s: int(s.strip(), 0),
                config['Robot']['RedBoard_port_ids'].split(','))
        )

        # Initialize the ROS 2 node with the robot name and node name
        super().__init__(f'{robot_name}_{node_name}')

        self.open_roboteq()
        #------------SUBSCRIBERS------------

        # Enable state subscriber
        self.enable_state_subscriber = self.create_subscription(String,
                                                                f'/{robot_name}/{ENABLE_STATE_TOPIC}',  # Replace with your actual topic name
                                                                self.enable_state_callback,
                                                                queue_length,)
        self.enable_state_subscriber  # Prevent unused variable warning

        # Command subscriber
        self.control_subscriber = self.create_subscription(Twist,
                                                           f'/{robot_name}/{control_topic}',
                                                           self.move_cmd_callback,
                                                           queue_length)
        self.encoder_timer = self.create_timer(encoder_timer_frequency, self.encoder_callback)
        # Timer to periodically check if the enable_state publisher is alive

        #-------------PUBLISHERS------------

        # Create publishers for each drive unit
        self.encoder_publisher = self.create_publisher(Int32MultiArray, f"/{robot_name}/{encoder_topic}", queue_length)
        self.liveliness_publisher = self.create_publisher(Bool, f"/Robot/{robot_name}", queue_length)
        # Pioneer ops course specific code
        sonar_topic             = config['Robot']['sonar_topic']
        self.sonar_distance     = []
        self.sonar_publisher    = self.create_publisher(UInt16MultiArray, f"/{robot_name}/{sonar_topic}", queue_length)
        self.sonar_timer        = self.create_timer(sonar_timer_frequency, self.sonar_callback)
        self.sonar_dists        = {"distances": [], "fresh": False}
        self.sonar_lock         = threading.Lock()
        self.open_sonar()
        self.sonar_thread       = threading.Thread(target=self.serial_read, daemon=True)
        self.sonar_stop         = threading.Event()
        self.sonar_thread.start()


        #------------ARM CONTROL___________
        self.arm_x_pos_gains    = float(config['Arm']['x_pos_gains'])
        self.arm_y_pos_gains    = float(config['Arm']['y_pos_gains'])
        self.arm_z_pos_gains    = float(config['Arm']['z_pos_gains'])
        self.arm_x_pose_gains   = float(config['Arm']['x_pose_gains'])
        self.arm_z_pose_gains   = float(config['Arm']['z_pose_gains'])
        gripper                 = bool(config['Arm']['gripper'])
        # Publisher for arm joint control
        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        # Action client for GripperCommand
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        while not self.gripper_client.wait_for_server(timeout_sec=1) and gripper:
            self.get_logger().warn("No gripper action server, skipping")
            

        self.arm_joint_positions = [0.0] * 4

        self.arm_joint_target = [0.0] * 4
        self.arm_joint_target[1:4] = ikne([0.2,0.1],0)
        self.arm_coordinates = [0.0,0.2,0.1,0]
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        self.arm_interval = 0.01
        self.arm_cmd_time = time.time()
        self.arm_timer = self.create_timer(0.01, self.send_arm_command)
        self.gripper_position = 0.0
        self.gripper_last_cmd = self.gripper_position
        self.gripper_max = 0.019
        self.gripper_min = -0.01

        self.joint_received = False

        self.max_delta = 0.02
        self.gripper_delta = 0.002
        self.last_command_time = time.time()

    def joint_state_callback(self, msg):
        if set(self.arm_joint_names).issubset(set(msg.name)):
            for i, joint in enumerate(self.arm_joint_names):
                index = msg.name.index(joint)
                self.arm_joint_positions[i] = msg.position[index]

        if 'rh_r1_joint' in msg.name:
            index = msg.name.index('rh_r1_joint')
            self.gripper_position = msg.position[index]
        if abs(self.gripper_position - self.gripper_last_cmd) < 1e-4:
            return



        self.joint_received = True
        #self.get_logger().info(
        #    f'Received joint states: {self.arm_joint_positions}, '
        #    f'Gripper: {self.gripper_position}'
        #)

    def send_arm_command(self):
        if self.state == "disabled":
            return
        now = time.time()
        #if (now - self.arm_cmd_time) < self.arm_interval:
        #    return
        arm_msg = JointTrajectory()
        arm_msg.joint_names = self.arm_joint_names
        max_step = 40 * self.arm_interval
        delta = np.clip(np.array(self.arm_joint_target) - np.array(self.arm_joint_positions), -max_step, max_step)
        target = self.arm_joint_positions + delta
        arm_target = JointTrajectoryPoint()
        arm_target.positions = target      
        arm_target.time_from_start = Duration(sec=0, nanosec=15_000_000)
        arm_msg.points = [arm_target]
        self.arm_publisher.publish(arm_msg)

    def send_gripper_command(self):
        goal_msg = GripperCommand.Goal()
        if abs(self.gripper_position - self.gripper_last_cmd) < 1e-4:
            return
        goal_msg.command.position = self.gripper_position
        goal_msg.command.max_effort = 10.0

        goal_handle = self.gripper_client.send_goal_async(goal_msg)
        self.gripper_last_cmd = self.gripper_position

    def find_ports(self):
        devices = {}
        for port in list_ports.comports():
            # for Roboteq
            Roboteq = port.vid==self.port_ids['Roboteq'][0] and port.pid==self.port_ids['Roboteq'][1]
            Redboard = port.vid==self.port_ids['RedBoard'][0] and port.pid==self.port_ids['RedBoard'][1]
            if Roboteq:
                devices["Roboteq"] = port.device
            elif Redboard:
                devices["Redboard"] = port.device
        return devices

    def open_roboteq(self):
        """Attempt to open the Roboteq port; retry until successful."""
        port_name = None
        self.get_logger().info("Looking for Roboteq controller…")
        while rclpy.ok():
            devices = self.find_ports()
            if 'Roboteq' in devices:
                port_name = devices['Roboteq']
                try:
                    self.roboteq = serial.Serial(
                        port=port_name,
                        baudrate=int(self.baudrate),
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        timeout=1
                    )
                    self.get_logger().info(f"Connected to Roboteq on {port_name}")
                    return
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to open {port_name}: {e}")
            time.sleep(1)

    
    def open_sonar(self):
        """Same pattern for your Redboard sonar port."""
        self.get_logger().info("Looking for Redboard sonar…")
        while rclpy.ok():
            devices = self.find_ports()
            if 'Redboard' in devices:
                try:
                    self.sonar_sensor = serial.Serial(devices['Redboard'], 9600, timeout=1)
                    self.get_logger().info(f"Connected to sonar on {devices['Redboard']}")
                    return
                except serial.SerialException as e:
                    self.get_logger().warn(f"Sonar open failed: {e}")
            time.sleep(1)

    def serial_read(self):
        buf = bytearray()
        while not self.sonar_stop.is_set() and rclpy.ok():
            try:
                buf += self.sonar_sensor.read(self.sonar_sensor.in_waiting or 1)
                while 0 in buf:                       # COBS delimiter
                    frame, buf = buf.split(b'\x00', 1)
                    try:
                        decoded   = decode(frame)
                        distances = struct.unpack(f'<{len(decoded)//2}H', decoded)
                        with self.sonar_lock:
                            self.sonar_dists["distances"] = distances
                            self.sonar_dists["fresh"] = True
                    except DecodeError:
                        pass
            except serial.SerialException as e:
                self.get_logger().error(f"Sonar read error: {e} — reconnecting")
                self.open_sonar()

    def enable_state_callback(self, msg):
        self.state = msg.data

    def arm_control(self, linear, angular, polar):
        
        if polar:
            arm_angle = [0.0] * 4
            # 2) polar adjustments
            r = np.sqrt(self.arm_coordinates[0]**2 + self.arm_coordinates[1]**2)

            theta = np.arctan2(self.arm_coordinates[0],self.arm_coordinates[1])
            r += linear.y * self.arm_y_pos_gains
            self.arm_coordinates[2] += linear.z * self.arm_z_pos_gains
            self.arm_coordinates[3] += angular.x * self.arm_x_pose_gains
            #print(pose)
            planar_angle = ikne([r,self.arm_coordinates[2]], self.arm_coordinates[3])
            arm_angle[0] = theta + angular.z * self.arm_z_pose_gains * 4
            
            if(planar_angle is None):
                self.arm_coordinates = fkne_3d(self.arm_joint_positions)
                return
            self.arm_coordinates[0] = r*np.sin(self.arm_joint_positions[0]) 
            self.arm_coordinates[1] = r*np.cos(self.arm_joint_positions[0]) 
            arm_angle[1:4] = planar_angle
        else:
            self.arm_coordinates[1] += linear.y * self.arm_y_pos_gains
            self.arm_coordinates[0] += angular.z * self.arm_z_pose_gains * 0.1
            self.arm_coordinates[2] += linear.z * self.arm_z_pos_gains
            self.arm_coordinates[3] += angular.x * self.arm_x_pose_gains
            arm_angle = ikne_3d(self.arm_coordinates[0:3], self.arm_coordinates[3])
            if arm_angle is None:
                self.arm_coordinates = fkne_3d(self.arm_joint_positions)
                return

        # 5) send the new joint
        arm_angle, valid = joint_limits(arm_angle)
        if not valid:
            self.arm_coordinates = fkne_3d(arm_angle)
        self.arm_joint_target = arm_angle

        # 6) gripper ← linear.x
        self.gripper_position = max(
            self.gripper_min,
            min(self.gripper_max,
                self.gripper_position + linear.x * self.gripper_delta)
        )
        self.send_gripper_command()

    def move_cmd_callback(self, msg):
        # Update linear and angular velocities from the Twist message
        self.linear_velocity['x'] = msg.linear.x
        self.linear_velocity['y'] = msg.linear.y
        self.linear_velocity['z'] = msg.linear.z
        self.angular_velocity['x'] = msg.angular.x
        self.angular_velocity['y'] = msg.angular.y
        self.angular_velocity['z'] = msg.angular.z
        left = right = 0
        if self.state == "skid":
            [left, right] = skid_steering_evaluation(self.linear_velocity, self.angular_velocity, self.motor_gain)
        elif self.state == "tank":
            [left, right] = tank_steering_evaluation(self.linear_velocity, self.motor_gain)
        elif self.state == "arm_polar":
            self.arm_control(msg.linear, msg.angular, True)
            self.roboteq.write(f"!G 1 {0}_".encode())
            self.roboteq.write(f"!G 2 {0}_".encode())
            return
        elif self.state == "arm_cart":
            self.arm_control(msg.linear, msg.angular, False)
            self.roboteq.write(f"!G 1 {0}_".encode())
            self.roboteq.write(f"!G 2 {0}_".encode())
            return
        else:
            return

        try:
            self.roboteq.write(f"!G 1 {left}_".encode())
            self.roboteq.write(f"!G 2 {right}_".encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Roboteq write error: {e} — attempting reconnect")
            self.open_roboteq()
            
    def encoder_callback(self):
        encoder_message = Int32MultiArray()

        try:
            self.roboteq.write(self.encoder_cmd.encode())
            #time.sleep(TIMER_CALLBACK_FREQUENCY_SECONDS)
            resp = self.roboteq.read_all().decode('utf-8', "ignore").split("\r")
            if len(resp) > 1 and ("C=" in resp[1]):
                resp = resp[1][2:].split(":")
                try:
                    encoder_message.data = [int(val) for val in resp]
                except (ValueError, IndexError) as e:
                    print(f"Retrying. Exception: {e}")
                self.encoder_publisher.publish(encoder_message)
        except serial.SerialException as e:
            self.get_logger().error(f"Roboteq comm error: {e} — reconnecting")
            self.open_roboteq()

    def sonar_callback(self):
        sonar_message = UInt16MultiArray()
        with self.sonar_lock:
            if self.sonar_dists["fresh"]:
                self.sonar_dists["fresh"] = False
                self.sonar_distance = self.sonar_dists["distances"]
        if not self.sonar_distance:
            return
        sonar_message.data = self.sonar_distance
        self.sonar_publisher.publish(sonar_message)



def main(args=None):
    rclpy.init(args=args)
    node = RoverProcessor()
    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(node)
    try:
        # Spin the executor to handle callbacks
        executor.spin()
    except KeyboardInterrupt:
        # just break out; let finally do the cleanup
        pass
    finally:
        # Clean up
        node.sonar_stop.set()
        node.sonar_thread.join(timeout=1)
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
