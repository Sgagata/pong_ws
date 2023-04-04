import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cam_input',
            executable='light_position',
            name='light_position'),
  ])