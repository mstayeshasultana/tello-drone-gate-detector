## Tello Autonomous Drone - Gate Racing
This project implements a fully autonomous navigation system for the Ryze Tello drone using ROS (Robot Operating System). The drone is programmed to navigate a complex racing arena by processing its live camera feed to detect, localize, and fly through a series of gates.




## 🎯 Project Objectives
The mission was to create a robust vision-based flight controller capable of:

#### Vision-Only Navigation: 
Entirely dependent on real-time image processing (no pre-mapped coordinates).
#### Dynamic Gate Traversal: 
Identifying and passing through gates of varying colors, shapes, sizes, and altitudes.



## 🛠️ Technical Implementation
Framework: ROS (Robot Operating System) for node communication and drone control.

#### Computer Vision: 
Developed a custom perception pipeline to handle color segmentation and geometric shape detection to calculate the drone's relative position to each gate.
#### Control Loop: 
Real-time PID or velocity commands sent via the Tello SDK based on visual feedback.

##  🏁 Results
The system successfully handles the "Gate Challenge" by prioritizing target gates based on visual proximity and executing smooth transitions between different gate heights and orientations.


## 📺 Project Demo

<div align="center">
  
  <p><b>Figure 1:</b> Tello drone identifying the green gate using HSV color filtering.</p>
</div>

![Tello Autonomous Racing Demo](https://github.com/user-attachments/assets/3317ff02-dea1-4bde-8631-8a13e501bd8a)

https://github.com/user-attachments/assets/3317ff02-dea1-4bde-8631-8a13e501bd8a




https://github.com/user-attachments/assets/aa33c754-b675-4768-a535-63f0d0515f03



## Group members:
1. Mst Ayesha Sultana
2. Abdullokh Orifjonov



## Instructions on how to run the project

### Terminal 1
#### Build the package
    mkdir -p ~/tello-drone-ws/src
    cd ~/tello-drone-ws/src
    git clone https://github.com/mstayeshasultana/tello-drone-gate-detector.git
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build


#### Launch the world in gazebo which has four green gates
    export GAZEBO_MODEL_PATH=$PWD/src/tello-drone-gate-detector/drone_racing_ros2/tello_ros/tello_gazebo/models:$GAZEBO_MODEL_PATH 
    source /usr/share/gazebo/setup.sh 
    ros2 launch tello_gazebo gate_detect_launch.py  

### Terminal 2
#### Fly the drone to detect gates and go towards it( For gazebo)
    source /opt/ros/galactic/setup.bash
    colcon build
    ros2 run gate_detector image_listener 

#### Fly the drone to detect gates and go towards it( For real scenario)
    source /opt/ros/galactic/setup.bash
    colcon build
    ros2 run gate_detector real_image_listener 
