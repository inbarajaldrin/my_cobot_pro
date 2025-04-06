
# my_cobot_pro

This ROS 2 package allows you to simulate and control the **MyCobot Pro 600** in RViz, perform inverse kinematics (IK), and use a camera plugin for line detection.

---

## Features

- RViz visualization and control of MyCobot Pro 600  
- Inverse kinematics solving using MoveIt  
- Camera plugin for line detection

---

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone git@github.com:inbarajaldrin/my_cobot_pro.git
```

### 2. Install Dependencies

Use `rosdep` to install all required dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package

```bash
colcon build --packages-select my_cobot_pro
```

### 4. Source the Workspace

```bash
source install/setup.bash
```

---

## Usage

### Launch RViz and MyCobot Pro

```bash
ros2 launch my_cobot_pro mycobot_pro_rviz.launch.py
```

### Run Inverse Kinematics Example

```bash
ros2 run my_cobot_pro ik_demo_node
```

### Start Camera Line Detection Node

```bash
ros2 run my_cobot_pro line_detection_node
```

> Make sure your camera topics and parameters are correctly set in the launch or YAML files.

---
