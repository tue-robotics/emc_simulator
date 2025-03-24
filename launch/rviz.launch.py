import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the RViz configuration file
    rviz_config_path = os.path.join(get_package_share_directory('emc_simulator'), 'rviz', 'mrc.rviz')

    return LaunchDescription([
        # Declare an argument to allow override of the RViz config file path
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path,
            description='Full path to the RViz config file'
        ),

        # Start RViz with the specified configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        ),
    ])
