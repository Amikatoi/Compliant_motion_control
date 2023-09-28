# UR5e Calligraphy Project

## Introduction
The art of calligraphy is a meticulous process that requires precision, style, and consistency. Traditionally the domain of human artisans, the potential to replicate this art using robotic systems offers intriguing possibilities in terms of scalability, consistency, and adaptability. This project aims to bridge the world of traditional calligraphy with modern robotics by harnessing the capabilities of the UR5e robotic arm to execute intricate Chinese and English calligraphy tasks.

## Idea
Calligraphy, both Chinese and English, has its roots deeply embedded in history, symbolizing tradition, culture, and artistry. The idea of automating this age-old art form, while retaining the precision and aesthetics of a skilled calligrapher, serves as the primary inspiration for this project.

## Objective
The primary objective is to explore the effectiveness and adaptability of the UR5e robotic arm in executing calligraphy tasks. The underlying research questions are (temporary):
1. How do varying surface textures impact the UR5e's performance in calligraphy tasks using Cartesian Impedance Control?
2. What is the system's adaptability to various calligraphy styles or writing tools? And, is there a need for significant recalibration when switching between writing styles?

## Methodology
The project leverages Cartesian Impedance Control, wherein the desired Cartesian pose of the robot’s end effector in the task space is inputted. This input assists in computing the required torque, which is then commanded to each joint of the robot, guiding the end effector along the predetermined trajectories.

To achieve the conversion of Chinese characters into robot-compatible coordinates, the project draws inspiration from methodologies proposed in references [1] and [2]. The process involves decomposing Chinese characters into individual strokes and subsequently mapping these strokes to a database containing robotic writing trajectories.

## Milestones
### Prototyping & Simulation (Target Completion: Sep 28)
The project's inaugural phase is centered around simulation. By establishing a robust simulated environment, we aim to lay the foundational groundwork. Here, we'll model a basic representation tailored for the calligraphy task, particularly focusing on crafting fundamental strokes. This simulation will serve as the testing bed, offering insights and guiding refinements.
### Real-World Implementation & Preliminary Testing (Target Completion: Nov 15)
Post-simulation, the subsequent phase involves breathing life into our project by transitioning to the actual UR5e setup. This stage is crucial, as it's here that our control algorithms will face real-world challenges. Initial tests will be executed to assess basic robotic movements, ensuring that the controller's implementation aligns with our objectives.
### Algorithm Deployment & Calligraphy Execution (Target Completion: Dec 27)
With a functioning UR5e setup in place, our attention will pivot to the intricate process of converting Chinese characters into robot-friendly coordinates. Drawing inspiration from methodologies in references [1] and [2], characters will be deconstructed into distinct strokes, which will then be mapped to a database brimming with robotic writing trajectories. The culmination of this phase will see the UR5e arm seamlessly inscribing characters, guided by our bespoke algorithms.
### Final Refinements, Comprehensive Testing, & Documentation (Target Completion: Feb 28)
As we approach the project's finale, our focus will intensify on refining and optimizing the system. Rigorous testing under varied scenarios will be conducted, ensuring adaptability and precision. Any identified gaps will be addressed, leading to necessary system tweaks. Concurrently, we'll embark on the documentation process, chronicling our journey and findings. This phase will also see the curation of a representation poster, encapsulating our project's essence.

In a nutshell, my project endeavors to craft a symphony between the age-old art of calligraphy and the modern marvel that is the UR5e robotic arm, setting the stage for a revolutionary exploration of tradition through the prism of technology.

[1]: https://doi.org/10.3390/app10238694
[2]: https://ieeexplore.ieee.org/abstract/document/8570842
