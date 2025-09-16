````markdown
TurtleBot3 A* Path Planning with Dynamic Obstacle Avoidance (ROS2)

This project implements an A* path planner as a ROS2 plugin for TurtleBot3 in Gazebo simulation.  
It demonstrates real-time path replanning when obstacles appear dynamically in the environment.

----------------------------------------------------------

Project Structure
tb3_ws/
├── src/
│   ├── tb3_astar_planner/     (A* path planning plugin)
│   ├── tb3_launch/            (Launch files for simulation)
│   └── tb3_world/             (World + Map files: YAML + PGM)
├── install/
├── build/
├── log/
└── README.md

----------------------------------------------------------

Features
* A* based path planner integrated with Nav2
* Runs on TurtleBot3 in Gazebo
* Dynamic replanning when obstacles are introduced
* Customizable environment (maps/worlds)

----------------------------------------------------------

Setup Instructions

1. Install ROS2 & TurtleBot3 Packages  
Make sure you have ROS2 Humble (or compatible) installed and TurtleBot3 simulation packages:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-nav2-bringup
````

2. Clone the Repository

```bash
cd ~
git clone https://github.com/<your-username>/tb3_ws.git
cd tb3_ws
```

3. Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

---

How to Run

Step 1: Launch TurtleBot3 in Gazebo

```bash
ros2 launch tb3_launch tb3_gz_nav.launch.py
```

Step 2: Launch Navigation Stack with A\* Planner
(Open a new terminal, source the workspace, and run:)

```bash
source ~/tb3_ws/install/setup.bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=$HOME/tb3_ws/src/tb3_world/maps/map_1758046643.yaml
```

Step 3: Send Navigation Goal
Open RViz2
Use the 2D Nav Goal tool to set a target position
The robot will:

1. Plan a path using A\*
2. Navigate to the goal
3. Replan dynamically if new obstacles are introduced in Gazebo

---

Demonstration

1. Robot navigates to a goal with no obstacles
2. An obstacle is added mid-path
3. The planner recomputes a new path and avoids the obstacle

---

Deliverables

1. Complete ROS2 Workspace (this repo) with launch files
2. Demonstration Video showing obstacle avoidance
3. One-Pager (Performance Analysis, Documentation, Challenges)

---

Performance Analysis

* A\* guarantees an optimal path (given grid discretization)
* Real-time replanning ensures safe navigation when obstacles appear
* Tested in Gazebo with TurtleBot3 Burger model

---

Challenges Faced

* Plugin integration with Nav2 required tuning
* Handling dynamic obstacles needed map updates + replanning triggers
* Real-time replanning introduces computational overhead for large maps

---

Author
Rishabh Jain
Graduate Engineer | Robotics & Autonomous Systems

```
