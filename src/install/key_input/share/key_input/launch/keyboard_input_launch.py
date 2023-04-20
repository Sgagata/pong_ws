from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='key_input',
            executable='key_input',
            name = 'key_input',
            prefix='xterm -e', #  Run the node in a separate xterm window. Install with `sudo apt install xterm` if needed.
        ),
    ])