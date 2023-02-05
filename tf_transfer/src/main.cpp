#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>

//global variables
std::array<double,12> effort;
std::array<double,12> position;
std::array<double,12> velocity;

void transferTF0(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF1(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF2(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF3(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF4(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF5(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF6(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF7(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF8(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF9(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF10(const control_msgs::JointControllerState::ConstPtr& msg);
void transferTF11(const control_msgs::JointControllerState::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "tf_transfer");
	effort.fill(0);
	position.fill(0);
	velocity.fill(0);

	std::array<std::string,12> joint_names = {
		"front_left_rolling_joint", 
		"front_left_pitching_joint", 
		"front_left_knee_joint", 
		"front_right_rolling_joint", 
		"front_right_pitching_joint", 
		"front_right_knee_joint",
		"back_left_rolling_joint",
		"back_left_pitching_joint",
		"back_left_knee_joint",
		"back_right_rolling_joint",
		"back_right_pitching_joint",
		"back_right_knee_joint"
	};
    ros::NodeHandle nh_sub;	
	std::array<ros::Subscriber, 12> sub;	
	sub.at(0) = nh_sub.subscribe("/testudog_controller/front_left_rolling_controller/state", 1, transferTF0);
	sub.at(1) = nh_sub.subscribe("/testudog_controller/front_left_pitching_controller/state", 1, transferTF1);
	sub.at(2) = nh_sub.subscribe("/testudog_controller/front_left_knee_controller/state", 1, transferTF2);
	sub.at(3) = nh_sub.subscribe("/testudog_controller/front_right_rolling_controller/state", 1, transferTF3);
	sub.at(4) = nh_sub.subscribe("/testudog_controller/front_right_pitching_controller/state", 1, transferTF4);
	sub.at(5) = nh_sub.subscribe("/testudog_controller/front_right_knee_controller/state", 1, transferTF5);
	sub.at(6) = nh_sub.subscribe("/testudog_controller/back_left_rolling_controller/state", 1, transferTF6);
	sub.at(7) = nh_sub.subscribe("/testudog_controller/back_left_pitching_controller/state", 1, transferTF7);
	sub.at(8) = nh_sub.subscribe("/testudog_controller/back_left_knee_controller/state", 1, transferTF8);
	sub.at(9) = nh_sub.subscribe("/testudog_controller/back_right_rolling_controller/state", 1, transferTF9);
	sub.at(10) = nh_sub.subscribe("/testudog_controller/back_right_pitching_controller/state", 1, transferTF10);
	sub.at(11) = nh_sub.subscribe("/testudog_controller/back_right_knee_controller/state", 1, transferTF11);

	ros::NodeHandle nh_pub;	
	ros::Publisher pub;
	pub = nh_pub.advertise<sensor_msgs::JointState>("/joint_states", 1);
	sensor_msgs::JointState joint_angle;
	std_msgs::Header head;
	ros::Rate loop_rate(25);
	int count = 0;

	while(ros::ok()){
		ros::Time t = ros::Time::now();
		joint_angle.name = {};
		joint_angle.effort = {};
		joint_angle.position = {};
		joint_angle.velocity = {};
		joint_angle.header.stamp.sec = t.toSec();
		joint_angle.header.stamp.nsec = t.toNSec();
		joint_angle.header.seq = count;

		for(int i = 0; i < 12; i++){
			joint_angle.name.push_back(joint_names.at(i));
			joint_angle.effort.push_back(effort.at(i));
			joint_angle.position.push_back(position.at(i));
			joint_angle.velocity.push_back(velocity.at(i));
		}

		std_msgs::String text;
		std::stringstream ss;
		ss << joint_angle << '\n';
		text.data = ss.str();
		ROS_INFO("%s", text.data.c_str());

		pub.publish(joint_angle);
		count ++;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void transferTF0(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(0) = msg->command;
	position.at(0) = msg->process_value;
	velocity.at(0) = msg->process_value_dot;
	ros::spinOnce();
}

void transferTF1(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(1) = msg->command;
	position.at(1) = msg->process_value;
	velocity.at(1) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF2(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(2) = msg->command;
	position.at(2) = msg->process_value;
	velocity.at(2) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF3(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(3) = msg->command;
	position.at(3) = msg->process_value;
	velocity.at(3) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF4(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(4) = msg->command;
	position.at(4) = msg->process_value;
	velocity.at(4) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF5(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(5) = msg->command;
	position.at(5) = msg->process_value;
	velocity.at(5) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF6(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(6) = msg->command;
	position.at(6) = msg->process_value;
	velocity.at(6) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF7(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(7) = msg->command;
	position.at(7) = msg->process_value;
	velocity.at(7) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF8(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(8) = msg->command;
	position.at(8) = msg->process_value;
	velocity.at(8) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF9(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(9) = msg->command;
	position.at(9) = msg->process_value;
	velocity.at(9) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF10(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(10) = msg->command;
	position.at(10) = msg->process_value;
	velocity.at(10) = msg->process_value_dot;
	ros::spinOnce();
}
void transferTF11(const control_msgs::JointControllerState::ConstPtr& msg){
	effort.at(11) = msg->command;
	position.at(11) = msg->process_value;
	velocity.at(11) = msg->process_value_dot;
	ros::spinOnce();
}




