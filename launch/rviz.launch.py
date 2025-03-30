import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of this launch file
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))

    # Set the default path to the RViz configuration file relative to this launch file
    default_rviz_config_path = os.path.join(launch_file_dir, '../rviz/mrc.rviz')

    return LaunchDescription([
        # Declare an argument to allow override of the RViz config file path
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config_path,
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