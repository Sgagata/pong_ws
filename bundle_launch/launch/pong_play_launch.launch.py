import launch
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription()

    window_server= Node (
        package = "pong_core",
        executable = "window_server"
    )

    left_bar= Node (
        package = "pong_core",
        executable = "left_light"
    )

    right_bar= Node (
        package = "pong_core",
        executable = "right_key"
    )

    ball = Node (
        package = "pong_core",
        executable = "ball"
    )

    collision = Node (
        package = "pong_core",
        executable = "collision_detection"
    )

    game_state = Node (
        package = "pong_core",
        executable = "game_state_publisher"
    )

    visualization = Node(
        package = "pong_visualization",
        executable="pong_visualization"
    )


    camera_node = Node (
        package = "image_tools",
        executable = "cam2image"
    )

    light_position = Node (
        package = "cam_input",
        executable = "light_position"
    )

    keyboard = Node(
        package = "key_input",
        executable="key_input",
        prefix='xterm -e'
    )

    ld.add_action(camera_node)
    ld.add_action(light_position)
    ld.add_action(window_server)
    ld.add_action(ball)
    ld.add_action(left_bar)
    ld.add_action(right_bar)
    ld.add_action(collision)
    ld.add_action(game_state)
    ld.add_action(keyboard)
    ld.add_action(visualization)


    return ld