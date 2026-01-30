from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multirobot_controller',
            executable='multirobot_controller',
            output='screen',
            emulate_tty = True,
            prefix = ''
        ),
        Node(
            package='joy',
            executable='joy_node'
        )
    ])
