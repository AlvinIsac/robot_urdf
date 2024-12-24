# Assignment for Experimental Robotics

## Prerequisites

Make sure you have the **ros2_aruco** package installed before proceeding.

---

## Steps to Run the Assignment

### 1. Clone the Repository

First, clone this package to your workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/AlvinIsac/robot_urdf.git
```

### 2. Build and Source the Package

Navigate to your workspace, build the package, and source it:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Running the Simulation

### Step 1: Launch the Gazebo World

Open the first terminal and run the following command:
```bash
ros2 launch robot_urdf gazebo_hw.launch.py
```

- **What Happens**:
  - Gazebo will launch with a custom world surrounded by 5 ArUco markers.
  - RViz2 will also launch.
  - A robot will spawn in the Gazebo environment.
  - The `aruco_node` will start for marker detection.

**Note**: **Wait for Gazebo to fully launch before proceeding.**

---

### Step 2: Run the Rotation Script

Open the second terminal and run this command:
```bash
ros2 run robot_urdf rotate_robot.py
```

- **What Happens**:
  - The robot will perform a full 360-degree rotation.
  - During the rotation, it scans all the marker IDs in the environment.
  - The script identifies the marker with the **lowest ID**.
  - The robot then rotates counterclockwise and stops at the lowest marker ID.

---

## Customization

Feel free to modify the Gazebo world to include different ArUco markers and test the system with your custom setup!

---


## Happy testing!

