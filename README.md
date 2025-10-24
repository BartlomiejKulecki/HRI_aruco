# HRI Aruco

### Simple Human-Robot Interaction using ArUco markers.

This is a ROS 2 package for controlling the UR robotic arm using the ArUco marker detected on webcam image.

## Requirements

- ROS 2 Jazzy
- install required Python packages using script:
    ```bash
    ./requirements.sh
    ```
- clone UR robotic arm simulation:
    ```bash
    git clone https://github.com/BartlomiejKulecki/Universal_Robots_ROS2_GZ_Simulation.git 
    ```


## Running

### Option 1: controlling the robot using ROS controller
Run robot simulation:
```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```
Run camera driver node:
```bash
ros2 run usb_cam usb_cam_node_exe
```

Run HRI node:
```bash
ros2 run hri_aruco hri_node
```
This node subscribes to the image topic (camera feed), detects ArUco marker using opencv, and depending on marker pose, sends goal commands to **joint trajectory action**.


### Option 2: controlling the robot using MoveIt interface
Run robot simulation:
```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```
Run camera driver node:
```bash
ros2 run usb_cam usb_cam_node_exe
```

Run HRI node:
```bash
ros2 run hri_aruco hri_node
```
This node subscribes to the image topic (camera feed), detects ArUco marker using opencv, and depending on marker pose, sends goal commands on **custom topic**. 

Use the node from the package ur_moveit_interface which subscribes to that custom topic and sends commands to **MoveIt**:
```bash
ros2 run ur_moveit_interface joint_goal_node 
```