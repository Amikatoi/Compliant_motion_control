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


## Hardware interfaces for controllers
Exposed interfaces per joint:

- `command_interfaces`: position, velocity, stiffness, damping
- `state_interfaces`: position, velocity

