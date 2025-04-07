# my_cobot_pro

This ROS 2 package allows you to simulate and control the **MyCobot Pro 600** in RViz, perform inverse kinematics (IK), and use a camera plugin for line detection.

---

## Features

- RViz visualization and control of MyCobot Pro 600  
- Inverse kinematics solving using MoveIt  
- Camera plugin for line detection and centerline following  
- Digital twin verification and real robot execution

---

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone git@github.com:inbarajaldrin/my_cobot_pro.git
```

### 2. Install Dependencies

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

## Setup for Line Detection

### 1. Capture and Crop Camera Image

After installing the workspace:

1. Launch your camera node or image viewer in simulation.
2. **Take a full-frame photo** of the camera's view and save it.
3. Crop the image to the region containing the **black centerline**.
4. Save the cropped image to:

```bash
~/ros2_ws/src/my_cobot_pro/camera/lab3.jpg
```

### 2. Run the Line Detection Script

```bash
python3 ~/ros2_ws/src/my_cobot_pro/scripts/black_line_border_crop.py
```

This script runs a line detection algorithm that outputs the coordinates of the black line.

> If the corners are incorrectly detected or included, update the `center_margin` parameter inside the script to adjust the region of interest. A value around `0.05` usually works well.

---

## Simulating and Verifying Motion

### 1. Launch Robot in RViz

```bash
ros2 launch my_cobot_pro demo.launch.py
```

### 2. Move Robot to Workspace Home Position

```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/JointTrajectory \
"{joint_names: ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'],
  points: [{positions: [0.704941, -2.266907, -2.281362, -0.19177, 1.567333, -0.22856],
            time_from_start: {sec: 3, nanosec: 0}}]}"
```

---

### 3. Run the IK Computation and Save Joint Trajectory

```bash
cd ~/ros2_ws/src/my_cobot_pro/scripts
python3 ik_crop_pub_joints.py
```

Make sure to set:

- `center_margin` to the correct value to crop image 
- `image_path` to the correct cropped image file
- `inter_num_points` to the number of interpolated centerline points

The output joint positions will be saved as:

```bash
~/ros2_ws/src/my_cobot_pro/data/joint_trajectory_log.json
```

---

### 4. Visualize the Digital Twin Movement in RViz

```bash
python3 ik_final_dt.py
```

This script sends the joint trajectory from the JSON file to the simulated robot for verification.

---

### 5. Send the Same Motion to the Real Robot

```bash
python3 ik_final_tcp.py
```

This script loads the same `joint_trajectory_log.json` and sends the motion commands to the physical MyCobot Pro 600 robot.

---
