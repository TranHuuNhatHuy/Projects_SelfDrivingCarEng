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

### 1. PID controller and parameter tuning

The full screen recording of the demo run could be seen in `assets/demo_run.mp4` of this repo. You can see that the car struggles to pass first 2 obstacles, but eventually succumbed to the last one. This is the furthest param tuning I could get.

During this project, I set the general params for motion planning as below (you can refer more in `starter_files/planning_params.h`):

```
#define P_NUM_PATHS 7 //1              // Num of paths (goals)
#define P_LOOKAHEAD_MIN 8.0            // m
#define P_LOOKAHEAD_MAX 20.0           // m
#define P_LOOKAHEAD_TIME 1.5           // s
#define P_GOAL_OFFSET 1.0              // m
#define P_ERR_TOLERANCE 0.1            // m
#define P_TIME_GAP 1.0                 // s
#define P_MAX_ACCEL 1.5                // m/s^2
#define P_SLOW_SPEED 1.0               // m/s
#define P_SPEED_LIMIT 3.0              // m/s
#define P_STOP_LINE_BUFFER 0.5         // m
#define P_STOP_THRESHOLD_SPEED 0.02    // m/s
#define P_REQ_STOPPED_TIME 1.0         // secs
#define P_LEAD_VEHICLE_LOOKAHEAD 20.0  // m
#define P_REACTION_TIME 0.25           // secs
#define P_NUM_POINTS_IN_SPIRAL 25      // Num of points in the spiral
#define P_STOP_THRESHOLD_DISTANCE \
  P_LOOKAHEAD_MIN / P_NUM_POINTS_IN_SPIRAL * 2  // m
```

Also, after days after nights trying to tune the PID params, as well as extensively searching for clues, I have finally came up with the PID parameters as below:

```C
// PID steering params
const double P_STEER_Kp = 0.3;
const double P_STEER_Ki = 0.0025;
const double P_STEER_Kd = 0.17;
const double P_STEER_MAX = 0.6;
const double P_STEER_MIN = -0.6;

// PID throttle params
const double P_THROTTLE_Kp = 0.21;
const double P_THROTTLE_Ki = 0.0006;
const double P_THROTTLE_Kd = 0.08;
const double P_THROTTLE_MAX = 1.0;
const double P_THROTTLE_MIN = -1.0;
```

As you can see from the demo, the vehicle demonstrates the ability to detect and avoid static obstacles, such as parked cars, within its lane. It successfully performs **nudge maneuvers** and **lane changes** when appropriate, ensuring a (smooth) and (almost) collision-free trajectory.

![alt text](image.png)

![alt text](image-1.png)

### 2. Evaluate and analyze the PID controller