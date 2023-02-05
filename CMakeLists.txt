cmake_minimum_required(VERSION 2.8.3)

project(testudog)

## Find ROS related packages/libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs control_msgs)

catkin_package()

find_package(roslaunch)
find_package(Eigen3 REQUIRED)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

## Build node
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gazebo_controller controller/src/main.cpp controller/src/testudog.cpp)
add_executable(gazebo_controller_plan controller/src/main_plan.cpp controller/src/testudog.cpp)
add_executable(tf_transfer tf_transfer/src/main.cpp)
target_link_libraries(gazebo_controller Eigen3::Eigen ${catkin_LIBRARIES})
target_link_libraries(gazebo_controller_plan Eigen3::Eigen ${catkin_LIBRARIES})
target_link_libraries(tf_transfer ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS sim2real/motorcontrol.py rrtstar/rrtstar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



