import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Package and Xacro configuration
    package_name = 'cartesian_controller_simulation'
    xacro_file_name = 'ur5_robot.urdf.xacro'

    # Get package path using launch_ros substitution
    pkg_path = FindPackageShare(package=package_name)
    xacro_file_path = PathJoinSubstitution([pkg_path, 'urdf', xacro_file_name])

    # Set GAZEBO_MODEL_PATH environment variable
    gazebo_models_path = PathJoinSubstitution([pkg_path, 'models'])
    set_gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_models_path)

    # Start Gazebo with GUI
    world_file_name = 'ur_setup.world'
    world_file_path = PathJoinSubstitution([pkg_path, 'worlds', world_file_name])
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Convert Xacro to URDF
    robot_description_content = Command(['xacro ', xacro_file_path])
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
    	package='robot_state_publisher',
    	executable='robot_state_publisher',
    	output='screen',
    	parameters=[robot_description, {"use_sim_time": True}]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ur5', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '1.594'],
        output='screen'
    )

    # Controllers to load
    controllers_to_load = [
        "joint_state_broadcaster",
        "cartesian_motion_controller",
        # Add more here
    ]

    # Chain loading of controllers after spawning entity
    previous_action = spawn_entity
    for controller in controllers_to_load:
        load_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', controller],
            output='screen'
        )
        # Register event handler to ensure sequential loading
        ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=previous_action,
                on_exit=[load_controller],
            )
        ))
        previous_action = load_controller

    ld.add_action(set_gazebo_model_path)
    ld.add_action(start_gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
