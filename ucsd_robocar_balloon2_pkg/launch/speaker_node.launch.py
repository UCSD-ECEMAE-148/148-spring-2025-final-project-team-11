from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ucsd_robocar_balloon2_pkg',
            executable='speaker_node',
            name='speaker_node',
            output='screen'
        )
    ])