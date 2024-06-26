# Cartesian Control for Chinese Calligraphy in ROS 2

## Introduction
The art of calligraphy is a meticulous process that requires precision, style, and consistency. Traditionally the domain of human artisans, the potential to replicate this art using robotic systems offers intriguing possibilities in terms of scalability, consistency, and adaptability. This project aims to bridge the traditional art of Chinese calligraphy with modern robotics technology. By leveraging robotic systems, particularly the UR5e robotic arm, this project serves as a resource for individuals interested in building upon or extending the capabilities of robotic systems for calligraphy or similar applications, specifically under ROS2. Whether you are a researcher, developer, or hobbyist, you can utilize the provided documentation, codebase, and methodologies to further explore the intersection of robotics and artistic expression.

## Objective
The project addresses a significant gap in the current robotics ecosystem, particularly within the context of ROS2, simulations, and the application of custom controllers, such as for the UR5e robotic arm. Some of the objectives are:

- Translate Character Stroke Data: Develop a system that translates character stroke data into Cartesian coordinates interpretable by the robot.
- Create Interface for Cartesian Information: Design an interface that conveys Cartesian information to the robotic arm's controller.
- Enable UR5e Robotic Arm for Calligraphy: Implement the necessary control mechanisms to enable the UR5e robotic arm to perform Chinese calligraphy strokes with precision and fidelity.

## Methodology
The methodology encompasses a process where the user inputs desired characters, which are then converted into SVG stroke data. These strokes are translated into Cartesian coordinates for the UR5e robotic arm equipped with a Cartesian controller [1]. It replicates the calligraphy motion by activating the controller to maneuver the brush and create the calligraphic strokes [2]. The function parses user-requested characters from a database, each characterized by an identifier, a sequence of SVG path data for stroke order, and a corresponding list of stroke medians. These elements are then transformed into Cartesian coordinates. The Cartesian Motion Controller is utilized for smooth motions [3]. Key to its flexibility are adjustable parameters:
- p and d gains: Control the responsiveness along each Cartesian axis
- Error scale: Adjust the controller's overall responsiveness
- Iterations per cycle: Determine the level of precision in tracking
<img src="/resources/Picture11.png" alt="System Diagram" width="500" height="800">

## Installation
Switch into the `src` folder of your current ROS2 workspace and
```bash
git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
cd ..
colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```
This builds the `cartesian_controllers` without its simulation environment.
The simulation is mostly relevant if you are just getting to know the `cartesian_controllers` and want to inspect how things work.
You can install it according to this [readme](cartesian_controller_simulation/README.md).

Now source your workspace again and you are ready to go.

## Getting started
For those who work on MacOS system, follow the tutorial below to install a virtual machine:
- [Virtual machine setup](macos_vm_setup/README.md)

To install the program that converts Chinese characters into Cartesian poses, follow:
- [Cartesian poses generator](calligraphy/README.md)

Then assuming you have the `cartesian_controller_simulation` package installed.
In a sourced terminal, call
```bash
ros2 launch cartesian_controller_simulation simulation.launch.py
```

This will start a simulated world in which you can inspect
and try things. Here are some quick tutorials with further details:
- [Cartesian controller simulation](cartesian_controller_simulation/README.md)
- [Cartesian motion controller](cartesian_motion_controller/README.md)

## Contributing
We welcome contributions from the robotics community to enhance and expand the capabilities of this project. Whether it is improving documentation, optimizing code, adding new features, or extending the helper package, your contributions are valuable in advancing the field of robotic calligraphy.

[1]: https://doi.org/10.3390/app10238694
[2]: https://ieeexplore.ieee.org/abstract/document/8570842
[3]: https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2
