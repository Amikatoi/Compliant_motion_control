# UR5e Calligraphy Project

## Introduction
The art of calligraphy is a meticulous process that requires precision, style, and consistency. Traditionally the domain of human artisans, the potential to replicate this art using robotic systems offers intriguing possibilities in terms of scalability, consistency, and adaptability. This project aims to bridge the traditional art of Chinese calligraphy with modern robotics technology. By leveraging robotic systems, particularly the UR5e robotic arm, this project serves as a resource for individuals interested in building upon or extending the capabilities of robotic systems for calligraphy or similar applications. Whether you're a researcher, developer, or hobbyist, you can utilize the provided documentation, codebase, and methodologies to further explore the intersection of robotics and artistic expression.

## Objective
1. Translate Character Stroke Data: Develop a system that translates character stroke data into Cartesian coordinates interpretable by the robot.
2. Create Interface for Cartesian Information: Design an interface that conveys Cartesian information to the robotic arm's controller.
3. Enable UR5e Robotic Arm for Calligraphy: Implement the necessary control mechanisms to enable the UR5e robotic arm to perform Chinese calligraphy strokes with precision and fidelity

## Methodology
The project leverages Cartesian Impedance Control, wherein the desired Cartesian pose of the robot’s end effector in the task space is inputted. This input assists in computing the required torque, which is then commanded to each joint of the robot, guiding the end effector along the predetermined trajectories. In the development of this project, I leveraged controller methods sourced from [3]. These methods, essential to the project's success, can be accessed and reviewed in the authors' entirety on their official GitHub repository. The repository provided valuable insights and a solid foundation, enabling a more streamlined and efficient implementation process.

To achieve the conversion of Chinese characters into robot-compatible coordinates, the project draws inspiration from methodologies proposed in references [1] and [2]. The process involves decomposing Chinese characters into individual strokes and subsequently mapping these strokes to a database containing robotic writing trajectories.


[1]: https://doi.org/10.3390/app10238694
[2]: https://ieeexplore.ieee.org/abstract/document/8570842
[3]: https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2

demon including simulation
problems that i have encountered and how i solve them
controller driver
understand what launch files look like and code
future projects
