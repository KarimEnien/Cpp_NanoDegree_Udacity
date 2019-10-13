#include "publisher.h"
#include "logger.h"
#include <memory> 


class controller
{

public:
controller(int rate, int cnt) : loopRate(rate) , number_count(cnt)
{};
void mainLoop();
void demoMovement();

private:
//Variable to control the publish rate of the node.
int loopRate;
//Variable of the number count for the joint angles.
float number_count;
};

void controller::mainLoop()
{
	//std::unique_ptr<logger> log(new logger);

	//While loop for incrementing number and publishing to topic /numbers
	while (ros::ok())
	{
		demoMovement();
	}
}

void controller::demoMovement(){

		// allocate a publlisher object and have it owned by std::unique_ptr, setting the buffer size ot 1000.
		std::unique_ptr<publisher> pub(new publisher(1000));
		//Create a rate object.
		ros::Rate loop_rate(loopRate);
		if (number_count <= 2)
		{
		//Created a Float64 message
		std_msgs::Float64 msg;

		//Inserted data to message header
		msg.data = number_count;

		//Printing message data
		ROS_INFO("%d",msg.data);

		//Publishing the topic 
		pub->joint_publisher_1.publish(msg);
		/*pub->joint_publisher_2.publish(msg);
		pub->joint_publisher_3.publish(msg);
		pub->joint_publisher_4.publish(msg);
		pub->joint_publisher_5.publish(msg);
		pub->joint_publisher_6.publish(msg);
		pub->joint_publisher_7.publish(msg);*/

		//Spinning once for doing the  all operation once
		ros::spinOnce();

		//Sleeping for sometime
		loop_rate.sleep();

		//Incrementing the count
		number_count = number_count + 0.1;
		}

		else if (number_count < 0)
		{

		//Created a Float64 message
		std_msgs::Float64 msg;

		//Inserted data to message header
		msg.data = number_count;

		//Printing message data
		ROS_INFO("%d",msg.data);

		//Publishing the topic 
		/*pub->joint_publisher_1.publish(msg);
		pub->joint_publisher_2.publish(msg);
		pub->joint_publisher_3.publish(msg);
		pub->joint_publisher_4.publish(msg);
		pub->joint_publisher_5.publish(msg);
		pub->joint_publisher_6.publish(msg);
		pub->joint_publisher_7.publish(msg);*/

		//Spinning once for doing the  all operation once
		ros::spinOnce();

		//Sleeping for sometime
		loop_rate.sleep();

		//Incrementing the count
		number_count = number_count - 0.1;
	}
}