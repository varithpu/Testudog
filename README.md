# Testudog
Quadruped robot simulation 

# Installation
1. Clone Testudog repository into your ROS workspace
```
git clone https://github.com/varithpu/Testudog.git
```
2. Build package with catkin_make
```
catkin_make
```
# Running the Simulation - Manual Control
1. Launch gazebo and robot model
```
roslaunch testudog testudog_launch.launch
```
2. Run robot motor controller 
```
rosrun testudog gazebo_controller
```

# Running the Simulation - Navigation in a Maze
1. Launch gazebo and robot model
```
roslaunch testudog testudog_planning.launch
```
2. Run RRT* Path Planner
```
rosrun testudog rrtstar.py 
```
3. Run robot motor controller 
```
rosrun testudog gazebo_controller_plan
```
# Screenshots
![alt text](https://github.com/varithpu/Testudog/blob/master/pics/pic3.png)
![alt text](https://github.com/varithpu/Testudog/blob/master/pics/pic4.png)
