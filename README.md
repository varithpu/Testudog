# Testudog
Quadruped robot simulation 

# Installation
1. Clone Testudog repository into your ROS workspace
2. Build package with catkin_make
```
catkin_make
```
# Running the Simulation
1. Launch gazebo and robot model
```
roslaunch testudog testudog_launch.launch
```
2. Run robot motor controller 
```
rosrun testudog gazebo_controller
```
# Changing Gaits

