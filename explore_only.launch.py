from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_pickup_delivery',
            executable='autonomous_explorer',
            name='explorer_node',
            output='screen',
        ),
    ])
