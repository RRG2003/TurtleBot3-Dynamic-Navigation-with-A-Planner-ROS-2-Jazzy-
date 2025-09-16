---

```markdown
# TurtleBot3 Navigation with Custom A* Planner (ROS 2 Jazzy)

## 📌 Overview
This project demonstrates autonomous navigation of a TurtleBot3 Burger robot in **Gazebo (Ignition)** using **ROS 2 Nav2** with a **custom A\* planner plugin**.  

The robot is capable of:
- Exploring an **unknown environment** with SLAM.  
- Planning paths using a custom **A\*** plugin.  
- Replanning paths dynamically when **new obstacles** appear.  

---

## 🗂 Repository Structure
```

tb3\_ws/
├── src/
│   ├── tb3\_astar\_planner/       # Custom A\* plugin
│   ├── tb3\_dynamic\_nav\_jazzy/   # Configs & launch files
│   └── turtlebot3\_gazebo/       # Gazebo simulation
├── install/
├── build/
├── log/
└── README.md

````

---

## ⚙️ Requirements
- **ROS 2 Jazzy** (Ubuntu 24.04 recommended)  
- **Gazebo (Ignition)**  
- TurtleBot3 and Nav2 packages:  
  ```bash
  sudo apt update
  sudo apt install ros-jazzy-turtlebot3* ros-jazzy-nav2* ros-jazzy-slam-toolbox
````

---

## 🔧 Build Instructions

1. Clone this repository into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone <your_repo_link> tb3_ws
   ```

2. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## 🚀 Running the Simulation

1. Launch Gazebo + Nav2 + RViz:

   ```bash
   ros2 launch tb3_dynamic_nav_jazzy tb3_gz_nav.launch.py
   ```

2. In RViz:

   * Set the **Initial Pose** using the 2D Pose Estimate tool.
   * Use **Nav2 Goal Tool** to set a navigation goal.

3. Watch the robot:

   * It will map the environment in real-time (SLAM Toolbox).
   * Plan a path using the **A\* plugin**.
   * Replan dynamically if new obstacles are introduced.

---

## 🎥 Demonstration

* **Without obstacles:** Robot successfully reaches goal.
* **With dynamic obstacles:** Robot detects new obstacle → replans path → safely continues to goal.

---

## 📑 Deliverables

* ✅ ROS 2 workspace (`tb3_ws`) with source code and configs.
* ✅ Custom A\* planner plugin (`tb3_astar_planner`).
* ✅ Launch file (`tb3_gz_nav.launch.py`).
* ✅ RViz configuration (`slam_nav_view.rviz`).
* ✅ [One Pager Report](one_pager.md) summarizing performance & challenges.
* ✅ Demo video showing navigation and obstacle avoidance.

---

## 🛠 Challenges & Notes

* **LiDAR noise** → tuned AMCL and costmap parameters.
* **TF synchronization issues** → adjusted transform tolerances.
* **Inflation radius tuning** was critical for balancing safety vs path feasibility.
* **Performance trade-off** → A\* provides optimal paths but is slower on large grids.

---

## ⚡ Quick Commands

For convenience, here’s a full workflow:

```bash
# Source workspace
cd ~/ros2_ws
source install/setup.bash

# Launch Gazebo + Nav2 + RViz
ros2 launch tb3_dynamic_nav_jazzy tb3_gz_nav.launch.py
```

Then in RViz:

1. Click **2D Pose Estimate** → set robot’s initial pose.
2. Click **Nav2 Goal** → set a destination.
3. Observe mapping + navigation.

---

## 👤 Author

Developed by **Rishabh Jain**
Project: TurtleBot3 Dynamic Navigation with A\* Planner (ROS 2 Jazzy)

```
