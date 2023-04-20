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
        executable = "left_bar_position"
    )

    right_bar= Node (
        package = "pong_core",
        executable = "right_bar_position"
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

    keyboard = Node(
        package = "key_input",
        executable="key_input",
        prefix='xterm -e'
    )



    ld.add_action(window_server)
    ld.add_action(ball)
    ld.add_action(left_bar)
    ld.add_action(right_bar)
    ld.add_action(collision)
    ld.add_action(game_state)
    ld.add_action(visualization)
    ld.add_action(keyboard)


    return ld