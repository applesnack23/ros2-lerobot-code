from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        Node(
            package='first_package',
            executable='move_circle',
            name='circle_publisher'
        ),

        Node(
            package='first_package',
            executable='read_pose',
            name='pose_subscriber'
        ),
    ])
