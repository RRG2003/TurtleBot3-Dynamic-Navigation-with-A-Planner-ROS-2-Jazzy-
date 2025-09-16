```markdown
# TurtleBot3 A* Path Planning with Dynamic Obstacle Avoidance (ROS2)

This project implements an **A\*** path planner as a ROS2 plugin for TurtleBot3 in Gazebo simulation.  
It demonstrates real-time **path replanning** when obstacles appear dynamically in the environment.

---

## 📂 Project Structure
```

tb3\_ws/
├── src/
│   ├── tb3\_astar\_planner/     # A\* path planning plugin
│   ├── tb3\_launch/            # Launch files for simulation
│   └── tb3\_maps/              # Map files (YAML + PGM)
├── install/
├── build/
├── log/
└── README.md

````

---

## 🚀 Features
- A* based path planner integrated with Nav2
- Runs on TurtleBot3 in Gazebo
- Dynamic replanning when obstacles are introduced
- Customizable environment (maps/worlds)

---

## 🛠️ Setup Instructions

### 1. Install ROS2 & TurtleBot3 Packages
Make sure you have ROS2 Humble (or compatible) installed and TurtleBot3 simulation packages:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-nav2-bringup
````

### 2. Clone the Repository

```bash
cd ~
git clone https://github.com/<your-username>/tb3_ws.git
cd tb3_ws
```

### 3. Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch the Simulation

* Launch TurtleBot3 in Gazebo:

```bash
ros2 launch tb3_launch tb3_world.launch.py
```

* Launch Navigation with A\* planner:

```bash
ros2 launch tb3_launch tb3_navigation.launch.py
```

### 5. Run the Planner

* Send a navigation goal via RViz2 (click "2D Nav Goal").
* The robot will plan a path using A\*.
* Introduce obstacles in Gazebo, and observe **dynamic replanning**.

---

## 📺 Demonstration

* **Step 1:** Robot navigates to a goal with no obstacles.
* **Step 2:** An obstacle is added mid-path.
* **Step 3:** The planner recomputes a new path and avoids the obstacle.

---

## 📑 Deliverables

1. **Complete ROS2 Workspace** (this repo) with launch files.
2. **Demonstration Video** showing obstacle avoidance.
3. **One-Pager** (Performance Analysis, Documentation, Challenges).

---

## 📊 Performance Analysis

* A\* guarantees an **optimal path** (given grid discretization).
* Real-time replanning ensures safe navigation when obstacles appear.
* Tested in Gazebo with TurtleBot3 Burger model.

---

## ⚠️ Challenges Faced

* Plugin integration with Nav2 required tuning.
* Handling dynamic obstacles needed map updates + replanning triggers.
* Real-time replanning introduces computational overhead for large maps.

---

## 👤 Author

**Rishabh Jain**
Graduate Engineer | Robotics & Autonomous Systems

```
