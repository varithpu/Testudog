#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <iostream>
#include "../include/testudog/testudog.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "gazebo_controller");
    ros::NodeHandle nh;
	std::array<std::string,12> command_topic = {
		"/testudog_controller/front_left_rolling_controller/command",
		"/testudog_controller/front_left_pitching_controller/command",
		"/testudog_controller/front_left_knee_controller/command",
		"/testudog_controller/front_right_rolling_controller/command",
		"/testudog_controller/front_right_pitching_controller/command",
		"/testudog_controller/front_right_knee_controller/command",
		"/testudog_controller/back_left_rolling_controller/command",
		"/testudog_controller/back_left_pitching_controller/command",
		"/testudog_controller/back_left_knee_controller/command",
		"/testudog_controller/back_right_rolling_controller/command",
		"/testudog_controller/back_right_pitching_controller/command",
		"/testudog_controller/back_right_knee_controller/command"
	};
	std::array<ros::Publisher,12> pub;
	for (int i = 0; i <= 11; i++){
		pub.at(i) = nh.advertise<std_msgs::Float64>(command_topic.at(i), 1);
	}
	robot::Testudog test;
	ros::Subscriber sub_imu = nh.subscribe("testudog_imu/body_orientation", 1, &robot::Testudog::get_imu, &test);
	ros::Subscriber sub_vel = nh.subscribe("cmd_vel", 1, &robot::Testudog::get_cmd_vel, &test);
	ros::Subscriber sub_state = nh.subscribe("gazebo/model_states", 1, &robot::Testudog::get_state, &test);
	std::array<std_msgs::Float64,12> command_msg;
	ros::Rate loop_rate(60);

    while(ros::ok()){
		test.reach_goal();
		double step_time = 0.5;
		double fwd_step = test.get_lin_vel()*step_time;
		double rot_step = test.get_ang_vel()*step_time;
		test.trot_planning(fwd_step, rot_step, 0.03, step_time);
		//test.push_up();
		int k = 0;
		for (int j = 0; j <= 3; j++){
			for (int i = 0; i <= 2; i++){
				command_msg.at(k).data = test.get_joint_angle()(i,j);
				pub.at(k).publish(command_msg.at(k));
				k = k + 1;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
	// std_msgs::String msg;
	// std::stringstream ss;
	// ss << test.imu_roll << '\n';
	// msg.data = ss.str();
	// ROS_INFO("%s", msg.data.c_str());



