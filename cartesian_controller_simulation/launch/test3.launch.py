from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os
distro = os.environ['ROS_DISTRO']
if distro in ['galactic', 'humble', 'iron']:
    spawner = "spawner"
else:  # foxy
    spawner = "spawner.py"


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Define the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('cartesian_controller_simulation'),
        'urdf',
        'ur5.urdf'
    )

    # Read the URDF file
    with open(urdf_file_path, 'r') as file:
        robot_description_content = file.read()

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cartesian_controller_simulation"), "config", "controller_manager.yaml",
        ]
    )

    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        output="both",
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ('cartesian_compliance_controller/target_frame', 'target_frame'),
            ('cartesian_force_controller/target_wrench', 'target_wrench'),
            ('cartesian_compliance_controller/target_wrench', 'target_wrench'),
            ('cartesian_force_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('cartesian_compliance_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ]
    )

    # Convenience function for easy spawner construction
    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable=spawner,
            output="screen",
            arguments=[name] + [a for a in args],
        )

    # Active controllers
    active_list = [
        "joint_state_broadcaster",
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    # Inactive controllers
    inactive_list = [
        "cartesian_compliance_controller",
        "cartesian_force_controller",
        "cartesian_motion_controller",
        "motion_control_handle",
        "joint_trajectory_controller",
        "invalid_cartesian_compliance_controller",
        "invalid_cartesian_force_controller",
    ]
    state = "--inactive" if distro in ['humble', 'iron'] else "--stopped"
    inactive_spawners = [
        controller_spawner(controller, state) for controller in inactive_list
    ]

    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Visualization
    #rviz_config = PathJoinSubstitution(
        #[FindPackageShare("cartesian_controller_simulation"), "etc", "robot.rviz"]
    #)
    #rviz = Node(
        #package="rviz2",
        #executable="rviz2",
        #name="rviz2",
        #output="log",
        #arguments=["-d", rviz_config]
    #)

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), '/launch/gzserver.launch.py']),
        launch_arguments={'world': PathJoinSubstitution([FindPackageShare('cartesian_controller_simulation'), 'worlds', 'ur_setup.world'])}.items(),
    )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "ur5",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "1.0", "-Y", "0.0"
        ],
        output="screen"
    )
    
    # Nodes to start
    nodes = (
        [robot_state_publisher, gazebo, spawn_entity]
        + active_spawners
        + inactive_spawners
    )

    return LaunchDescription(declared_arguments + nodes)
