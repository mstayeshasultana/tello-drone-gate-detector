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


<img width="935" height="473" alt="Screenshot 2026-01-31 213804" src="https://github.com/user-attachments/assets/bb0bc231-b93d-4462-bd1b-c4737500870a" />

<img width="670" height="528" alt="Screenshot 2026-01-31 213754" src="https://github.com/user-attachments/assets/a4bbc9b5-dd07-44d3-bfa1-1d21c4247ab4" />

<div align="center">
  <video src="https://github.com/user-attachments/assets/56ef73a7-b582-49ac-871d-7a51587c973b" 
    width="100%" 
    controls="true" 
    muted="muted" 
    autoplay="true" 
    loop="true">
    Your browser does not support the video tag.
  </video>
</div>


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

#### 🛠️ Tech Stack (Drone Gate Detector)

* **Framework:** ROS 2 (Galactic) for node communication and drone control.
* **Computer Vision:** OpenCV and Python for the perception pipeline, color segmentation, and geometric shape detection.
* **Languages:** C++, Python.
* **Simulation & Tools:** Gazebo for 3D environment simulation and Docker for containerization.

    
