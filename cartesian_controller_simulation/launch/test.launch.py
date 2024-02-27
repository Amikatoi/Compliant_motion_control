import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Package and URDF configuration
    package_name = 'cartesian_controller_simulation'
    urdf_file_name = 'ur5.urdf'
    controller_config_file_name = 'controllers_manager.yaml'

    # Get package path
    pkg_path = get_package_share_directory(package_name)
    urdf_file_path = os.path.join(pkg_path, 'urdf', urdf_file_name)
    controller_config_path = os.path.join(pkg_path, 'config', controller_config_file_name)
    
    # Determine the ROS distribution
    distro = os.environ['ROS_DISTRO']
    spawner = "spawner" if distro in ['galactic', 'humble', 'iron'] else "spawner.py"

    # Set GAZEBO_MODEL_PATH environment variable
    gazebo_models_path = os.path.join(pkg_path, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_models_path)

    # Read the URDF file
    with open(urdf_file_path, 'r') as file:
        robot_description_content = file.read()
    robot_description = {'robot_description': robot_description_content}

    # World file
    world_file_name = 'ur_setup.world'
    world_file_path = os.path.join(pkg_path, 'worlds', world_file_name)
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), '/launch/gzserver.launch.py']),
        launch_arguments={'world': world_file_path}.items(),
    )

    # Robot State Publisher
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
        arguments=['-entity', 'ur5', '-topic', '/robot_description', '-x', '0.0', '-y', '0.0', '-z', '1.594', '-Y', '0.0'],
        output='screen'
    )
    
    # Declare 'headless' Launch Argument
    declare_headless_arg = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        description='Set to "true" to run Gazebo in headless mode.'
    )
    
    # Start Gazebo client (GUI)
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

#########################################################################################
    # Controller spawners
    #controller_spawners = []

    # Active controllers
    #active_list = ["joint_state_broadcaster"]
    #for controller in active_list:
        #controller_spawners.append(Node(
            #package="controller_manager",
            #executable=spawner,
            #arguments=['--controller-manager', '/controller_manager', '--activate', controller],
            #output="screen",
        #))

    # Inactive controllers
    #inactive_list = [
        #"cartesian_compliance_controller",
        #"cartesian_force_controller",
        #"cartesian_motion_controller",
        #"joint_trajectory_controller",
    #]
    #for controller in inactive_list:
        #controller_spawners.append(Node(
            #package="controller_manager",
            #executable=spawner,
            #arguments=['--controller-manager', '/controller_manager', '--activate', controller, '--inactive'],
            #output="screen",
        #))
#########################################################################################

    # Launch Description
    ld = LaunchDescription([
        set_gazebo_model_path,
        DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        declare_headless_arg,
        gazebo,
        robot_state_publisher,
        start_gazebo_client_cmd,
        spawn_entity
    ])

    return ld
