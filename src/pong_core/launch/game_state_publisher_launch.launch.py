import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pong_core',
            executable='game_state_publisher',
            name='game_state_publisher'),
  ])