# Cartesian Controller Simulation
This package provides a simulated robot for controller development and testing in Gazebo.

## Getting started
In a sourced terminal, run
```bash
ros2 launch cartesian_controller_simulation simulation.launch.py
```

You can call
```bash
ros2 control list_controllers
```
to get a list of controllers currently managed by the `controller_manager`.
All of them can be activated and tested in the simulator.
In contrast to `ROS1`, these controllers are nodes and you can also see them with `ros2 node list`.


## Troubleshooting
Some issues might occur during the process of simulation, mostly due to the change of simulation platform (from Mujoco to Gazebo). Sections below provide one of the solutions towards their corresponding problems.

### Controller node conflict
if you encounter the problem below:

<img src="/resources/12.png" alt="p1" width="650" height="250">
In the launch file, ros2_control_node (or any other manually written control node) is in conflict with gazebo_ros2_control. gazebo_ros2_control runs automatically based on hardware plugin in ros2 control tag in xacro file. This built-in control node can control controllers instead of ros2_control_node when working with Gazebo. To rewrite the launch file as well as the urdf file, follow the video:

[USING ROS2 WTH YOUR CUSTOM ROBOT](https://www.youtube.com/watch?v=EosEikbZhiM).


### robot_description is empty
After launching the robot in Gazebo, if the desired controller cannot be configured due to the *robot_description* being empty as shown below:

<img src="/resources/13.png" alt="p1" width="750" height="500">


## Hardware interfaces for controllers
Exposed interfaces per joint:

- `command_interfaces`: position, velocity, stiffness, damping
- `state_interfaces`: position, velocity

