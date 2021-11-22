# Basics of mobile robotics - Final project

## Thymio - Vision Guided Navigation



> **Students : Cécile Chavane, Yucef Grebici, Camille Guillaume, Théo Hermann**
>
> (Names in alphabetical order)


### Overview

This project aims to combine vision, path planning, local navigation, and filtering to maneuver a Thymio robot on a map towards a goal.

For the implementation, the image of the experimental site is first captured by the webcam. The necessary map information, including the robot pose, map, static obstacles, and the goal position, is extracted in a real-time fashion by utilizing the classic image processing techniques. Afterward, the Visibility Graph algorithm computes the optimal path. It sends the instructions for the global controller of the Thymio robot, which gives instructions to the motors to follow the optimal path. If Thymio detects an obstacle ahead via the horizontal proximity sensors, local navigation will take over the robot's movement to avoid collisions.

