import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'swarm_bringup'
    pkg_path = get_package_share_directory(pkg_name)
    config_path = os.path.join(pkg_path, "config", "swarm_controller.yaml")
    world_path = os.path.join(pkg_path, 'worlds', 'random_obstacles.world')

    # 1. Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={'gui': 'true', 'world': world_path}.items()
    )

    # --- HELPER FUNCTION TO GENERATE NODES FOR ONE ROBOT ---
    def generate_robot_nodes(name, x, y):
        # Use COMMAND to force a fresh xacro process (Prevents "Cross-Talk")
        xacro_file = os.path.join(pkg_path, "urdf", "swarm_robot.urdf.xacro")
        robot_desc = ParameterValue(
            Command(['xacro ', xacro_file, ' robot_name:=', name, ' config_file:=', config_path]),
            value_type=str
        )

        state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=name,
            output="screen",
            parameters=[{"robot_description": robot_desc, "use_sim_time": True, "frame_prefix": f"{name}/"}]
        )

        spawn_entity = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", name, "-topic", f"/{name}/robot_description", "-x", str(x), "-y", str(y), "-z", "0.2"],
            output="screen"
        )

        # We return the DIFF DRIVE spawner separately so we can use it as a trigger!
        diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", f"/{name}/controller_manager"],
        )

        joint_state_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", f"/{name}/controller_manager"],
        )

        # Return the list of nodes AND the trigger node
        return [state_pub, spawn_entity, joint_state_spawner, diff_drive_spawner], diff_drive_spawner

    # ==========================================================
    # 2. DEFINE THE SEQUENCE (THE CHAIN REACTION)
    # ==========================================================
    
    # --- ROBOT 1 ---
    r1_nodes, r1_trigger = generate_robot_nodes("robot1", "-6.0", "0.0")

    # --- ROBOT 2 (Waits for Robot 1's Diff Drive to finish) ---
    r2_nodes, r2_trigger = generate_robot_nodes("robot2", "-6.0", "3.0")
    
    spawn_robot2_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=r1_trigger,  # Wait for R1 Diff Drive to exit (finish loading)
            on_exit=r2_nodes           # Then spawn R2
        )
    )

    # --- ROBOT 3 (Waits for Robot 2's Diff Drive to finish) ---
    r3_nodes, r3_trigger = generate_robot_nodes("robot3", "-5.0", "-3.0")
    
    spawn_robot3_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=r2_trigger,  # Wait for R2 Diff Drive to exit
            on_exit=r3_nodes           # Then spawn R3
        )
    )

    # --- BRAINS (Wait for Robot 3 to finish) ---
    def create_brain(name):
        return Node(
            package='swarm_bringup', executable='swarm_node', 
            name=f'brain_{name}', namespace=name, 
            parameters=[{'robot_name': name}]
        )

    launch_brains_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=r3_trigger,
            on_exit=[create_brain('robot1'), create_brain('robot2'), create_brain('robot3')]
        )
    )

    # 3. LAUNCH DESCRIPTION
    # We only "actively" launch Gazebo and Robot 1. 
    # The rest are chained events.
    ld = LaunchDescription([gazebo])
    
    for node in r1_nodes:
        ld.add_action(node)
        
    ld.add_action(spawn_robot2_event)
    ld.add_action(spawn_robot3_event)
    ld.add_action(launch_brains_event)

    return ld
