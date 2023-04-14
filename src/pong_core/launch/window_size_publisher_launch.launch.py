import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pong_core',
            executable='window_size_publisher',
            name='window_size_publisher'),
  ])