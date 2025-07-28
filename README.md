Patrol FSM – TurtleBot Simulation in ROS 2

This ROS 2 project implements a Finite State Machine (FSM) for a simulated TurtleBot. The robot autonomously patrols a Gazebo world by navigating between checkpoints, inspecting each location, avoiding obstacles, and returning to a dock when the simulated battery level is low.

FSM States

* Navigating → Drives toward a checkpoint
* Station-Inspect → Rotates in place for inspection
* Transmit Panorama → Placeholder state (no actual image capture in this version)
* Blocked → Waits when an obstacle is detected
* Return-Dock → Returns to the docking station when battery is low

Installation & Build Instructions

1. Clone the repository into your ROS 2 workspace:

   
   cd ~/rosbot_ws/src
   git clone <your-repo-url>  # Replace with your actual GitHub repo


2. Build the workspace:

   cd ~/rosbot_ws
   colcon build
   source install/setup.bash

3. (Optional) Install any dependencies:

   sudo apt update
   sudo apt install ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs

Running the Simulation

1. Launch the Gazebo world (make sure sim\_launch is properly configured):

   ros2 launch sim_launch empty_world.launch.py

2. In a new terminal (and source the workspace):

   source ~/rosbot_ws/install/setup.bash
   ros2 run patrol_fsm fsm_node

The robot will automatically begin patrolling the checkpoints and responding to simulated battery and obstacle events.

FSM Log File

State transitions are saved with timestamps at:

/tmp/fsm_log.csv

Each line records: timestamp, previous state, new state.

Related Concepts

This project demonstrates:

* Finite State Machines (FSMs) in robotic control
* Obstacle detection using LaserScan
* Patrol logic and simulated battery depletion
* Integration with Gazebo simulation using ROS 2
