import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    emc_simulator_share_dir = get_package_share_directory("emc_simulator")

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=os.path.join(emc_simulator_share_dir, "data", "defaultconfig.json"),
        description="Path to the simulation config file"
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(emc_simulator_share_dir, "data", "heightmap_metadata.yaml"),
        description="Path to the map file"
    )

    robot_config = os.path.join(
        emc_simulator_share_dir,
        'config',
        'simbot_config.yaml'
    )
    
    # Node for the map server
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}],
        output="screen"
    )


    # Global parameter server node
    global_param_node= Node(
        package="emc_simulator",
        executable="global_parameter_server",
        name="global_parameter_server",
        parameters= [robot_config]
    )

    # Simulator node
    simulator = Node(
        package="emc_simulator",
        executable="pico_simulator",
        arguments=["--config", LaunchConfiguration("config")],
        output="screen",
        parameters=[os.path.join(emc_simulator_share_dir, "simconfig", "simbot_config.yaml")]
    )

    # Start the lifecycle manager for map_server
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'autostart': True},
                    {'node_names': ['map_server']} 
        ]
    )

    # Delay lifecycle manager to ensure map_server is ready first
    delayed_lifecycle_manager = TimerAction(
        period=1.0,  
        actions=[lifecycle_manager_cmd]
    )

    # Launch Description
    return LaunchDescription([
        config_arg,
        map_arg,
        map_server,
        delayed_lifecycle_manager,
        global_param_node,
        simulator
    ])
