import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'swarm_bringup'
    pkg_path = get_package_share_directory(pkg_name)
    config_path = os.path.join(pkg_path, "config", "swarm_controller.yaml")
    
    # LOAD THE OBSTACLE WORLD
    world_file_name = 'random_obstacles.world'
    world_path = os.path.join(pkg_path, 'worlds', world_file_name)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={
            'gui': 'true',
            'world': world_path
        }.items()
    )

    def create_robot_instance(name, x, y):
        xacro_file = os.path.join(pkg_path, "urdf", "swarm_robot.urdf.xacro")
        
        # ENABLE CAMERA FOR EVERYONE
        robot_desc = xacro.process_file(
            xacro_file, 
            mappings={
                "robot_name": name, 
                "config_file": config_path,
                "enable_camera": "true" 
            }
        ).toxml()

        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=name,
            output="screen",
            parameters=[{
                "robot_description": robot_desc,
                "use_sim_time": True,
                "frame_prefix": f"{name}/" 
            }]
        )

        spawn = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", name, 
                "-topic", f"/{name}/robot_description", 
                "-x", str(x), "-y", str(y), "-z", "0.2" 
            ],
            output="screen"
        )

        jsb_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", f"/{name}/controller_manager"],
        )

        diff_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", f"/{name}/controller_manager"],
        )

        return [rsp, spawn, jsb_spawner, diff_spawner]

    # ====================================================
    # 1. Robot 1 (Immediate) - Center
    # ====================================================
    robot1_nodes = create_robot_instance("robot1", "-8.0", "0.0")  
    
    # ====================================================
    # 2. Robot 2 (Wait 10s) - Left
    # ====================================================
    robot2_nodes = TimerAction(
        period=10.0,
        actions=create_robot_instance("robot2", "-8.0", "2.0")
    )
    
    # ====================================================
    # 3. Robot 3 (Wait 20s) - Right
    # ====================================================
    robot3_nodes = TimerAction(
        period=20.0,
        actions=create_robot_instance("robot3", "-8.0", "-2.0")
    )

    return LaunchDescription([
        gazebo,
        *robot1_nodes, # Unpack R1 list
        robot2_nodes,  # TimerAction (No *)
        robot3_nodes   # TimerAction (No *)
    ])
