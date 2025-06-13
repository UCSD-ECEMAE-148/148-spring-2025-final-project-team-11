from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ucsd_robocar_balloon2_pkg',
            executable='balloon_node',
            name='balloon_node',
            parameters=[{
                'device_name': '18443010612A721200',
            }],
            output='screen'
        )
    ])