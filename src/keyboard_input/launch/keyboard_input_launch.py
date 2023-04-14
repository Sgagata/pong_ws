from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_input',
            executable='keyboard_input',
            prefix='xterm -e', #  Run the node in a separate xterm window. Install with `sudo apt install xterm` if needed.
        ),
    ])