import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="trackbar",
            node_executable="talker",
            node_name="talker",
            output='screen',
            ),

        launch_ros.actions.Node(
            package="trackbar",
            node_executable="listener",
            node_name="listener",
            output='screen',
            ),
        ])