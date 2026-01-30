import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, UInt16MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import os
import configparser
import sys
from collections import deque
import statistics
import re
import numpy as np
from sensor_msgs.msg import JointState

MAINTAIN = False    # a debug macro for those who are computationally challenged and is reading this code for the first time
                    # enable it to see what the member vars hold
CONFIG_FILE = "config.cfg"  # name of config file
TIMER_CALLBACK_FREQUENCY_SECONDS = 0.05
REFRESH_FREQUENCY_SECONDS = 1
DEFAULT_NAME_WIDTH = 5

# Text Styles
RESET            = "\033[0m"   # Reset all attributes

BOLD             = "\033[1m"   # Bold (increased intensity)
FAINT            = "\033[2m"   # Faint (decreased intensity)
ITALIC           = "\033[3m"   # Italic (slanted text)
UNDERLINE        = "\033[4m"   # Underline
BLINK_SLOW       = "\033[5m"   # Blink, low frequency
BLINK_RAPID      = "\033[6m"   # Blink, high frequency
REVERSE          = "\033[7m"   # Reverse video (swap fg/bg)
CONCEAL          = "\033[8m"   # Conceal (hidden)
STRIKETHROUGH    = "\033[9m"   # Strike-through (crossed-out)
OVERLINE         = "\033[53m"  # Overline (rarely supported)

# ── Style “Off” / Resets ───────────────────────────────────
BOLD_OFF             = "\033[22m"  # Normal intensity
ITALIC_OFF           = "\033[23m"  # Not italic
UNDERLINE_OFF        = "\033[24m"  # Not underlined
BLINK_OFF            = "\033[25m"  # Blink off
REVERSE_OFF          = "\033[27m"  # Positive image
REVEAL               = "\033[28m"  # Reveal (not concealed)
STRIKETHROUGH_OFF    = "\033[29m"  # Not crossed out
OVERLINE_OFF         = "\033[55m"  # No overline

# Foreground
# Standard Colours
FG_BLACK        = "\033[30m"
FG_RED          = "\033[31m"
FG_GREEN        = "\033[32m"
FG_YELLOW       = "\033[33m"
FG_BLUE         = "\033[34m"
FG_MAGENTA      = "\033[35m"
FG_CYAN         = "\033[36m"
FG_WHITE        = "\033[37m"
# Foreground Bright Colours
FG_BBLACK       = "\033[90m"
FG_BRED         = "\033[91m"
FG_BGREEN       = "\033[92m"
FG_BYELLOW      = "\033[93m"
FG_BBLUE        = "\033[94m"
FG_BMAGENTA     = "\033[95m"
FG_BCYAN        = "\033[96m"
FG_BWHITE       = "\033[97m"
# Custom (Extended 8bit)
FG_ORANGE       = "\033[38;5;208m"
FG_ROSE         = "\033[38;5;211m"
FG_TEAL         = "\033[38;5;30m"
FG_GRAY         = "\033[38;5;245m"

# Background
# Standard Colours
BG_BLACK        = "\033[40m"
BG_RED          = "\033[41m"
BG_GREEN        = "\033[42m"
BG_YELLOW       = "\033[43m"
BG_BLUE         = "\033[44m"
BG_MAGENTA      = "\033[45m"
BG_CYAN         = "\033[46m"
BG_WHITE        = "\033[47m"
# Bright Colours
BG_BBLACK       = "\033[100m"
BG_BRED         = "\033[101m"
BG_BGREEN       = "\033[102m"
BG_BYELLOW      = "\033[103m"
BG_BBLUE        = "\033[104m"
BG_BMAGENTA     = "\033[105m"
BG_BCYAN        = "\033[106m"
BG_BWHITE       = "\033[107m"
# Custon (Extended 8 bit)
BG_ORANGE       = "\033[48;5;208m"
BG_GRAY         = "\033[48;5;245m"
BG_DARKBLUE     = "\033[48;5;17m"

# Utility
# Bold Colours
BOLD_RED        = BOLD + FG_RED
BOLD_GREEN      = BOLD + FG_GREEN
BOLD_YELLOW     = BOLD + FG_YELLOW
BOLD_BLUE       = BOLD + FG_BLUE
BOLD_MAGENTA    = BOLD + FG_MAGENTA
BOLD_CYAN       = BOLD + FG_CYAN
BOLD_ORANGE     = BOLD + FG_ORANGE
# Underlined Colours
UL_RED          = UNDERLINE + FG_RED
UL_CYAN         = UNDERLINE + FG_CYAN
UL_YELLOW       = UNDERLINE + FG_YELLOW
UL_ORANGE       = UNDERLINE + FG_ORANGE
UL_GREEN        = UNDERLINE + FG_GREEN
UL_MAGENTA      = UNDERLINE + FG_MAGENTA

class MultiSensorFilter:
    """
    Median‐then‐EMA filter that adapts to any number of input channels.
    """

    def __init__(self, window_size: int = 5, alpha: float = 0.3):
        self.window_size = window_size
        self.alpha = alpha
        # will hold a deque for each channel
        self.windows = []  # type: list[deque[int]]
        # will hold last EMA value for each channel
        self.ema = []      # type: list[float]

    def filter(self, values: list[int]) -> list[int]:
        # if channel count changed, reinitialize buffers
        if len(values) != len(self.windows):
            self.windows = [deque(maxlen=self.window_size) for _ in values]
            # start EMA at the first reading
            self.ema = [float(v) for v in values]

        out = []
        for i, v in enumerate(values):
            # 1) append to rolling window, compute median
            self.windows[i].append(v)
            m = statistics.median(self.windows[i])

            # 2) update EMA
            self.ema[i] += self.alpha * (m - self.ema[i])

            # 3) collect as int
            out.append(int(self.ema[i]))

        return out


class Multirobot_Controller(Node):

    ROBOT_RE = re.compile(r"^/Robot/([^/]+)")
    def __init__(self):
        super().__init__('Multirobot_Controller')
        # Get config file dir
        config_file_path = os.getcwd() 
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE))
        # Read config file 
        config = configparser.ConfigParser()
        if not config.read(config_file_path):
            self.get_logger().error(f"Config file {CONFIG_FILE} not found or invalid.")
            rclpy.shutdown()
            return
        
        # initialise config vars from config file for readability
        self.command_publish_topic       = config['Node']['command_publisher_topic']
        self.command_subscriber_topic    = config['Node']['command_subscriber_topic'] 
        self.queue_length           = int(config['Node']['queue'])
        self.display                = bool(config['Node']['display'])
        self.topic_name             = config['Topics']['topic_name']
        self.publisher_values       = ["Neutral", "Teach", "Play"]
        #self.kinematics_values      = ["skid", "tank"]
        self.kinematics_values      = ["skid", "tank", "arm_polar", "arm_cart"]
        #self.kinematics_values      = ["skid", "arm_cart"]
        self.enable_publisher_dict  = dict()

        self.robot_list = []
        self.cmd_publishers        = {}
        self.cmd_subscribers       = {}
        self.sonar_subscribers     = {}
        self.robot_states          = {}
        self.robot_sonar           = {}
        
        self.enable_button          = int(config['Joy']['enable_button'])
        self.enable_all_button      = int(config['Joy']['enable_all_button'])
        self.last_robot             = 0
        self.cycle_robots_button    = int(config['Joy']['cycle_robots_button'])
        self.cycle_mode_button      = int(config['Joy']['cycle_mode_button'])
        self.cycle_kinematics_button= int(config['Joy']['cycle_kinematics_button'])
        self.linear_x_axis          = int(config['Joy']['linear_x_axis'])
        self.angular_z_axis         = int(config['Joy']['angular_z_axis'])
        self.right_x_axis           = int(config['Joy']['right_x_axis'])
        self.linear_gain            = float(config['Joy']['linear_gain'])
        self.angular_gain           = float(config['Joy']['angular_gain'])
        self._name_width            = DEFAULT_NAME_WIDTH
        self.current_robot          = 0
        self.robot_toggle_flag      = False
        self.mode_toggle_flag       = False
        self.kinematics_toggle_flag = False
        self.enable_toggle_flag     = False
        self.mm_per_box             = int(config['Sonar']['mm_per_box'])
        self.sonar_max_val          = int(config['Sonar']['sonar_max_val'])
        self.boxes                  = int(self.sonar_max_val/self.mm_per_box)
        self.window_size            = int(config['Filter']['window_size'])
        self.alpha                  = float(config['Filter']['alpha'])
        #self.sonar_filter = MultiSensorFilter(window_size=window_size, alpha=alpha)
        self.sonar_filter = {}
        self.gripper_close_trigger = int(config['Arm']['gripper_close_trigger'])
        self.gripper_close_button = int(config['Arm']['gripper_close_button'])
        self.gripper_state = False
        self.gripper_toggle_flag = False
        self.arm_linear_y_axis = int(config['Arm']['linear_y_axis'])
        self.arm_linear_z_axis = int(config['Arm']['linear_z_axis'])
        self.arm_angular_x_axis = int(config['Arm']['angular_x_axis'])
        self.arm_angular_z_axis = int(config['Arm']['angular_z_axis'])

        self._mode_width  = 8          # "Neutral", "Manual", …
        self._state_width = 5          # "True", "False"
        self._kin_width   = 12         # "skid", "tank", …

        self.joint_subscription = None 
        self.arm_joint_positions = [0.0] * 4
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        names = self.arm_joint_names + ["Gripper"]
        self.name_width = max(len(n) for n in names)
        self._angle_width = 9
        self.gripper_position = 0.0

        self.joystick_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            self.queue_length)
        self.joystick_subscriber

        self.refresh_robots()
        self.timer = self.create_timer(TIMER_CALLBACK_FREQUENCY_SECONDS, self.timer_callback)
        self.refresh_timer = self.create_timer(REFRESH_FREQUENCY_SECONDS, self.refresh_callback)
        print("\033[2J",flush=True)
        self.print_all_robot_states()

    def __del__(self):
        sys.stdout.flush()

    def joint_state_callback(self, msg):
        if set(self.arm_joint_names).issubset(set(msg.name)):
            for i, joint in enumerate(self.arm_joint_names):
                index = msg.name.index(joint)
                self.arm_joint_positions[i] = msg.position[index]

        if 'rh_r1_joint' in msg.name:
            index = msg.name.index('rh_r1_joint')
            self.gripper_position = msg.position[index]

    def refresh_robots(self):
        """Discover robots by scanning topics, and add/remove as needed."""
        # 1) find all “/RobotXXX” prefixes
        topics = self.get_topic_names_and_types()
        found = {t.split('/')[2] for (t, _) in topics if t.startswith("/Robot/")}
        arm_found = { name for (name, _) in topics if name == '/joint_states' }
        if arm_found:
            self.joint_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        else:
            self.arm_joint_positions = [0.0] * 4
            self.gripper_position = 0
            self.destroy_subscription(self.joint_subscription)
            self.joint_subscription = None
        # 2) add new robots
        for robot in sorted(found - set(self.robot_list)):
            #self.get_logger().info(f"Discovered robot → {robot}")
            self.robot_list.append(robot)
            self._name_width = max(len(f"({i}) {name}") for i, name in enumerate(self.robot_list))
            # init state & data
            self.robot_states[robot] = {
                'Mode': self.publisher_values[0],
                'Enabled': False,
                'kinematics': self.kinematics_values[0]
            }
            self.enable_publisher_dict[robot] = self.create_publisher(String, f"/{robot}/{self.topic_name}", self.queue_length)
            self.robot_sonar[robot] = []
            self.sonar_filter[robot] = MultiSensorFilter(window_size=self.window_size, alpha=self.alpha)
            # create cmd pub/sub
            self.cmd_publishers[robot] = self.create_publisher(
                Twist, f"/{robot}/{self.command_publish_topic}", self.queue_length)
            self.cmd_subscribers[robot] = self.create_subscription(
                Twist, f"/{robot}/{self.command_subscriber_topic}",
                self.create_command_callback(robot),
                self.queue_length)
            # sonar subscription
            self.sonar_subscribers[robot] = self.create_subscription(
                UInt16MultiArray, f"/{robot}/sonar",
                self.create_sonar_callback(robot),
                self.queue_length
            )
            self.print_all_robot_states()

        # 3) remove robots that vanished
        for robot in set(self.robot_list) - found:
            #self.get_logger().info(f"Robot went offline → {robot}")
            self.robot_list.remove(robot)
            self._name_width = max(len(f"({i}) {name}") for i, name in enumerate(self.robot_list)) if self.robot_list else DEFAULT_NAME_WIDTH
            # tear down pubs/subs
            self.destroy_publisher(self.enable_publisher_dict[robot])
            del self.enable_publisher_dict[robot] 
            self.destroy_publisher(self.cmd_publishers.pop(robot))
            self.destroy_subscription(self.cmd_subscribers.pop(robot))
            self.destroy_subscription(self.sonar_subscribers.pop(robot))
            self.robot_states.pop(robot)
            self.robot_sonar.pop(robot)
            self.sonar_filter.pop(robot)
            self.print_all_robot_states()

    def sonar_bar(self, value_mm: int) -> str:
        filled = max(0, min(value_mm // self.mm_per_box, self.boxes))
        empty  = self.boxes - filled
        # colour-code: green >600 mm, yellow 301-600, red ≤300
        if value_mm > int(self.sonar_max_val):
            col = FG_BLUE
        elif value_mm > int(self.sonar_max_val*(2/3)):
            col = FG_GREEN
        elif value_mm > int(self.sonar_max_val*(1/3)):
            col = FG_YELLOW
        else:
            col = FG_RED
        value = BLINK_RAPID + FG_RED + " Out of Bounds" + RESET if (value_mm > self.sonar_max_val) else f" {value_mm:4d} mm"
        return (
            col + "▉" * filled + RESET + FAINT + FG_BLACK + "▉" * empty + RESET
            + value
        )

    def format_joints(self):
        tmpl = "{name:<" + str(self.name_width) + "} = {angle:>" + str(self._angle_width) + ".4f}"
        parts = []
        # 3. format each joint
        for name, pos in zip(self.arm_joint_names, self.arm_joint_positions):
            angle = np.degrees(pos)
            colored_name  = BOLD+FG_YELLOW + tmpl.format(name=name,  angle=angle).split(" = ")[0] + RESET
            colored_angle = FG_WHITE + tmpl.format(name=name,  angle=angle).split(" = ")[1] + RESET
            parts.append(f"{colored_name} = {colored_angle}")
        # and the gripper
        grip_angle = np.degrees(self.gripper_position)
        colored_name  = BOLD+FG_YELLOW + tmpl.format(name="Gripper", angle=grip_angle).split(" = ")[0] + RESET
        colored_angle = FG_ROSE + tmpl.format(name="Gripper", angle=grip_angle).split(" = ")[1] + RESET
        parts.append(f"{colored_name} = {colored_angle}")
        return "\t|\t".join(parts)

    def format_mode(self, mode):
        match mode:
            case "Neutral":
                return f"{UL_YELLOW}{mode}{RESET}"
            case "Teach":
                return f"{UL_CYAN}{mode}{RESET}"
            case "Play":
                return f"{UL_MAGENTA}{mode}{RESET}"
            case _:
                return ""

    def format_enabled(self, enabled):
        match enabled:
            case False:
                return f"{UL_RED}{enabled}{RESET}"
            case True:
                return f"{UL_GREEN}{enabled}{RESET}"
            case _:
                return ""

    def format_kinematics(self, kinematics):
        match kinematics:
            case "skid":
                return f"{UL_ORANGE}{kinematics}{RESET}"
            case "tank":
                return f"{UL_ORANGE}{kinematics}{RESET}"
            case "arm_polar":
                return f"{UL_ORANGE}{kinematics}{RESET}"
            case "arm_cart":
                return f"{UL_ORANGE}{kinematics}{RESET}"
            case _:
                return ""

    def print_all_robot_states(self):
        if not self.display:
            return

        # clear & home
        print("\033[2J\033[H", end="", flush = False)
        out_lines = []

        for idx, name in enumerate(self.robot_list):
            st = self.robot_states[name]

            # main status row (same alignment code you had)
            raw    = f"({idx}) {name}"
            pad    = " " * max(0, self._name_width - len(raw))
            name_s = f"{(UNDERLINE if idx == self.current_robot else '')}{BOLD}{FG_BLUE}{raw}{RESET}{pad}"
            mode_s = f"{self.format_mode(st['Mode']):<{self._mode_width}}"
            en_s   = f"{self.format_enabled(st['Enabled']):<{self._state_width}}"
            kin_s  = f"{self.format_kinematics(st['kinematics']):<{self._kin_width}}"

            out_lines.append(
                f"{name_s} | Mode : {mode_s}"
                f" | State : {en_s}"
                f" | Kinematics : {kin_s}"
            )

            # ── sonar panel (only when selected) ───────────────────────────────
            if idx == self.current_robot:
                sonar_vals = self.robot_sonar[name]
                for x in range(len(self.robot_sonar[name])):
                    out_lines.append(f"\tSonar{x+1} {self.sonar_bar(sonar_vals[x])}")
                if self.joint_subscription:
                    out_lines.append(f"\t{self.format_joints()}")


        # dump everything at once, then flush
        print("\n".join(out_lines), flush=True)

    def kinematics(self, msg):
        messages = [Twist() for _ in self.robot_list]
        for x in range(len(self.robot_list)):
            match self.robot_states[self.robot_list[x]]['kinematics']:
                case 'skid':
                    messages[x].linear.x = self.linear_gain*msg.axes[self.linear_x_axis]
                    messages[x].angular.z = self.angular_gain*msg.axes[self.angular_z_axis]
                case 'tank':
                    messages[x].linear.x = self.linear_gain*msg.axes[self.linear_x_axis]
                    messages[x].linear.y = self.linear_gain*msg.axes[self.right_x_axis]
                case 'arm_polar':
                    messages[x].linear.x = -1.0 if self.gripper_state else msg.axes[self.gripper_close_trigger]
                    messages[x].linear.y = msg.axes[self.arm_linear_y_axis] 
                    messages[x].linear.z = msg.axes[self.arm_linear_z_axis]
                    messages[x].angular.x = msg.axes[self.arm_angular_x_axis]
                    messages[x].angular.z = msg.axes[self.arm_angular_z_axis]
                case 'arm_cart':
                    messages[x].linear.x = -1.0 if self.gripper_state else msg.axes[self.gripper_close_trigger]
                    messages[x].linear.y = msg.axes[self.arm_linear_y_axis] 
                    messages[x].linear.z = msg.axes[self.arm_linear_z_axis]
                    messages[x].angular.x = msg.axes[self.arm_angular_x_axis]
                    messages[x].angular.z = msg.axes[self.arm_angular_z_axis]

        return messages

    def joy_callback(self, msg):
        if not self.robot_list:
            return
        messages = [Twist() for _ in self.robot_list]
        # cycle through selected robot
        if msg.buttons[self.cycle_robots_button] and not self.robot_toggle_flag:
            prev_robot_index = self.current_robot
            self.current_robot = (self.current_robot+1)%len(self.robot_list)
            self.robot_toggle_flag = True
        elif not msg.buttons[self.cycle_robots_button]:
            self.robot_toggle_flag = False

        # cycle though mode of selected robot
        if msg.buttons[self.cycle_mode_button] and not self.mode_toggle_flag:
            current_mode = self.robot_states[self.robot_list[self.current_robot]]['Mode']  
            self.robot_states[self.robot_list[self.current_robot]]['Mode'] = self.publisher_values[(self.publisher_values.index(current_mode)+1)%len(self.publisher_values)] 
            self.mode_toggle_flag = True
        elif not msg.buttons[self.cycle_mode_button]:
            self.mode_toggle_flag = False

        # cycle through kinematics mode
        if msg.buttons[self.cycle_kinematics_button] and not self.kinematics_toggle_flag:
            current_kinematics = self.robot_states[self.robot_list[self.current_robot]]['kinematics']  
            self.robot_states[self.robot_list[self.current_robot]]['kinematics'] = self.kinematics_values[(self.kinematics_values.index(current_kinematics)+1)%len(self.kinematics_values)] 
            self.kinematics_toggle_flag = True
        elif not msg.buttons[self.cycle_kinematics_button]:
            self.kinematics_toggle_flag = False

        if msg.buttons[self.gripper_close_button] and not self.gripper_toggle_flag:
            self.gripper_state = not self.gripper_state
            self.gripper_toggle_flag = True
        elif not msg.buttons[self.gripper_close_button]:
            self.gripper_toggle_flag = False



        # enable robot though dead mans switch
        if msg.buttons[self.enable_button] and not self.enable_toggle_flag:
            if self.current_robot != self.last_robot:
                self.robot_states[self.robot_list[self.last_robot]]['Enabled'] = False
                self.last_robot = self.current_robot
            self.robot_states[self.robot_list[self.current_robot]]['Enabled'] = not self.robot_states[self.robot_list[self.current_robot]]['Enabled']
            self.enable_toggle_flag = True
        elif msg.buttons[self.enable_all_button] and not self.enable_toggle_flag:
            for robot in self.robot_states.values():
                robot['Enabled'] = not robot['Enabled']
                self.enable_toggle_flag = True
        elif not msg.buttons[self.enable_all_button] and not msg.buttons[self.enable_button]:
            self.enable_toggle_flag = False

        for x in range(len(self.robot_list)):
            if self.robot_states[self.robot_list[x]]['Enabled'] and self.robot_states[self.robot_list[x]]['Mode'] == self.publisher_values[2]: # if in play mode
                messages = self.kinematics(msg)
                self.cmd_publishers[self.robot_list[x]].publish(messages[x])

        self.print_all_robot_states()

    def create_command_callback(self, robot):
        # Define a callback for a specific robot
        def command_callback(msg):
            message = Twist()
            if self.robot_states[robot]['Enabled'] and self.robot_states[robot]['Mode'] == self.publisher_values[1]: # if in Teach mode
                message = msg
            elif self.robot_states[robot]['Enabled'] and self.robot_states[robot]['Mode'] == self.publisher_values[2]: # if in play mode
                return
            self.cmd_publishers[robot].publish(message)
        return command_callback

    def create_sonar_callback(self, robot):
        # Define a callback for a specific robot
        def sonar_callback(msg):
            self.robot_sonar[robot] = self.sonar_filter[robot].filter(msg.data)
        return sonar_callback


    def timer_callback(self):
        for robot in self.robot_list:
            msg = String()
            msg.data = self.robot_states[robot]['kinematics']
            if not self.robot_states[robot]["Enabled"]:
                msg.data = "disabled"   
            self.enable_publisher_dict[robot].publish(msg)
    
    def refresh_callback(self):
        self.refresh_robots()
        self.print_all_robot_states()

def main(args=None):
    # clear screen
    print("\033[2J",end="")
    rclpy.init(args=args)
    node = Multirobot_Controller()
    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(node)
    try:
        # Spin the executor to handle callbacks
        executor.spin()
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt, shutting down...")
    finally:
        # Clean up
        executor.shutdown()
        node.destroy_node()
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except rclpy.RCLError:
            print("Error: rcl_shutdown already called.")
        print("\033[2J\033[H", end="", flush = False)


if __name__ == '__main__':
    main()
