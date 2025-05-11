# tello-drone-gate-detector

## Group members:
1. Mst Ayesha Sultana
2. Abdullokh Orifjonov



## Instructions on how to run the project

### Terminal 1
#### Build the package
    mkdir -p ~/tello-drone-gate-detector/src
    cd ~/tello-drone-gate-detector/src
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
