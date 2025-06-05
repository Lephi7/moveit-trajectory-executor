import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('moveit_trajectory_executor')
    config_file = os.path.join(package_dir, 'config', 'live_pose_config.yaml')
    
    # Check if config file exists
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    # Declare launch arguments for runtime parameter overrides
    pose_rate_arg = DeclareLaunchArgument(
        'pose_update_rate_ms',
        default_value='50',
        description='Pose update rate in milliseconds'
    )
    
    show_joints_arg = DeclareLaunchArgument(
        'show_joint_values',
        default_value='false',
        description='Whether to show joint values'
    )
    
    # Node with both config file and runtime overrides
    live_pose_node = Node(
        package='moveit_trajectory_executor',
        executable='live_pose',
        name='live_pose',
        parameters=[
            config_file,  # Load from YAML file
            {
                # Allow runtime overrides
                'pose_update_rate_ms': LaunchConfiguration('pose_update_rate_ms'),
                'show_joint_values': LaunchConfiguration('show_joint_values'),
            }
        ],
        output='screen',
        emulate_tty=True,  # Better output formatting
    )
    
    return LaunchDescription([
        pose_rate_arg,
        show_joints_arg,
        live_pose_node
    ])