import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pong_viz',
            executable='pong_visualization',
            name='pong_visualization'),
  ])