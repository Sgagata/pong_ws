import launch
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription()

    camera_node = Node (
        package = "image_tools",
        executable = "cam2image"
    )

    light_position = Node (
        package = "cam_input",
        executable = "light_position"
    )

    light_position_subsciber = Node (
        package = "cam_input",
        executable = "light_position_subscriber"
    )

    ld.add_action(camera_node)
    ld.add_action(light_position)
    ld.add_action(light_position_subsciber)

    return ld
