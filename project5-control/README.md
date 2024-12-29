# PROJECT - CONTROL & TRAJECTORY TRACKING FOR AUTONOMOUS VEHICLES

## I. Introduction

The **Control & Trajectory Tracking for Autonomous Vehicles** project is the 5th and final mandatory project of [Udacity Nanodegree: Self Driving Car Engineer](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013). It focuses on implementing and tuning control algorithms to enable precise trajectory tracking. In this project, students design a **Proportional-Integral-Derivative (PID)** controller to guide an autonomous vehicle along a predefined trajectory in a simulated environment.

Using the CARLA simulator, an industry-standard tool, students test their PID controller's ability to maintain accurate and stable vehicle control while tracking a series of waypoints. The project emphasizes understanding the interplay between the PID components (Proportional, Integral, and Derivative) and tuning their parameters to optimize the vehicle's performance.

This project offers hands-on experience with fundamental control systems, preparing students for real-world challenges in autonomous vehicle development by integrating theoretical principles with practical simulation-based testing.

## II. Tasks

### 1. Design PID controller

- Develop a **Proportional-Integral-Derivative (PID)** controller in C++ to compute the control inputs needed for precise trajectory tracking.
- Understand and implement the mathematical framework behind the PID algorithm, focusing on its proportional, integral, and derivative components.

### 2. Integrate with CARLA simulator

- Set up a simulation environment using the CARLA simulator.
- Connect the PID controller to the CARLA simulator to enable the autonomous vehicle to receive control commands and provide feedback.

### 3. Parameter tuning

- Optimize the PID parameters for accurate and stable trajectory tracking.
- Balance the trade-offs between overshooting, settling time, and steady-state error.

### 4. Testing and performance visualization

- Generate plots to analyze the controller's performance, including metrics like tracking error, control input behavior, and stability over time.
- Record a video from the CARLA simulator to visually demonstrate the controller's success in tracking the trajectory.

## III. Results

