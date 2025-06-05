from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_trajectory_executor',
            executable='test_trajectory',
            output='screen',
        )
    ])