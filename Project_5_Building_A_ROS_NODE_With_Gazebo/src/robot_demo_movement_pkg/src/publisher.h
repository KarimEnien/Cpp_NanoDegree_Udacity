//roscpp Core header 
#include "ros/ros.h"
//Header of Int32 standard message
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"


class publisher
{
public:
	publisher(int bufSz): bufferSize(bufSz)
	{
	//Created a node handle object
	ros::NodeHandle node_obj;
	//Created a publisher object
	joint_publisher_1 = node_obj.advertise<std_msgs::Float64>(topic_1,bufferSize);
	joint_publisher_2 = node_obj.advertise<std_msgs::Float64>(topic_2,bufferSize);
	joint_publisher_3 = node_obj.advertise<std_msgs::Float64>(topic_3,bufferSize);
	joint_publisher_4 = node_obj.advertise<std_msgs::Float64>(topic_4,bufferSize);
	joint_publisher_5 = node_obj.advertise<std_msgs::Float64>(topic_5,bufferSize);
	joint_publisher_6 = node_obj.advertise<std_msgs::Float64>(topic_6,bufferSize);
	joint_publisher_7 = node_obj.advertise<std_msgs::Float64>(topic_7,bufferSize);
	}

private:
	int bufferSize = 0;
	ros::Publisher joint_publisher_1;
	ros::Publisher joint_publisher_2;
	ros::Publisher joint_publisher_3;
	ros::Publisher joint_publisher_4;
	ros::Publisher joint_publisher_5;
	ros::Publisher joint_publisher_6;
	ros::Publisher joint_publisher_7;

	std::string topic_1 = "/seven_dof_arm/joint1_position_controller/command";
	std::string topic_2 = "/seven_dof_arm/joint2_position_controller/command";
	std::string topic_3 = "/seven_dof_arm/joint3_position_controller/command";
	std::string topic_4 = "/seven_dof_arm/joint4_position_controller/command";
	std::string topic_5 = "/seven_dof_arm/joint5_position_controller/command";
	std::string topic_6 = "/seven_dof_arm/joint6_position_controller/command";
	std::string topic_7 = "/seven_dof_arm/joint7_position_controller/command";
	friend class controller;
};