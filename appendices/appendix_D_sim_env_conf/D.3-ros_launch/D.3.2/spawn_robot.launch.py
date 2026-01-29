import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'swarm_bringup'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Define Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_fixed.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'swarm_controller.yaml') # Updated to use the correct config
    
    # --- NEW: Define the World File Path ---
    world_file = os.path.join(pkg_share, 'worlds', 'random_obstacles.world')

    # 2. Process Xacro
    # We pass 'robot_name' so the camera plugin gets a unique name (prevents crashes)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, 
                 ' config_file:=', controller_config,
                 ' robot_name:=my_bot']), 
        value_type=str
    )

    # 3. Start Gazebo with YOUR WORLD
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'gui': 'true',
            'world': world_file  # <--- Loads the random obstacles world
        }.items(), 
    )

    # 4. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                   '-z', '0.5'], 
        output='screen'
    )

    # 5. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 6. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # 7. Startup Events
    delay_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        delay_joint_state,
        delay_diff_drive,
    ])
