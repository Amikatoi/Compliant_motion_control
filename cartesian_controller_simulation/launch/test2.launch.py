import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShar

def generate_launch_description():
    ld = LaunchDescription()

    # Package and URDF configuration
    package_name = 'cartesian_controller_simulation'
    urdf_file_name = 'ur5.urdf'
    world_file_name = 'ur_setup.world'

    # Get package path
    pkg_path = get_package_share_directory(package_name)
    urdf_file_path = os.path.join(pkg_path, 'urdf', urdf_file_name)
    world_file_path = os.path.join(pkg_path, 'worlds', world_file_name)

    # Set GAZEBO_MODEL_PATH environment variable
    gazebo_models_path = os.path.join(pkg_path, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_models_path)

    # Start Gazebo with GUI and specified world
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Read the URDF file
    with open(urdf_file_path, 'r') as file:
        robot_description_content = file.read()
    	robot_description = {'robot_description': robot_description_content}
        
    robot_state_publisher = Node(
    	package='robot_state_publisher',
    	executable='robot_state_publisher',
    	output='screen',
    	parameters=[robot_description]
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
