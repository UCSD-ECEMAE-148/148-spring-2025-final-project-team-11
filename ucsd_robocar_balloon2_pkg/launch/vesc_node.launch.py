from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ucsd_robocar_balloon2_pkg',
            executable='vesc_node',
            name='vesc_node',
            output='screen'
        )
    ])