#include "publisher.h"
#include "logger.h"
#include <memory> 


class controller
{

public:
controller(int rate, float cnt) : loopRate(rate) , number_count(cnt) {};
controller(){};
void mainLoop();
void demoMovement();
void setHomePoisition(const float &val);
void getPositionCallback(const std_msgs::String::ConstPtr& msg);


private:
//Variable to control the publish rate of the node.
int loopRate;
//Variable of the number count for the joint angles.
float number_count;
std_msgs::Float64 home_val;
};

void controller::mainLoop()
{

	setHomePoisition(0.0);
	demoMovement();
	std::cout << "Still in the while loop" << std::endl;
}

void controller::demoMovement()
{
		std::unique_ptr<logger> log(new logger);
		// allocate a publlisher object and have it owned by std::unique_ptr, setting the buffer size ot 1000.
		std::unique_ptr<publisher> pub(new publisher(1000));
		//Create a rate object.
		ros::Rate loop_rate(loopRate);
		// Creating a string to write to output file
		std::string str;
		// Creating the main while loop in the app.
		while(ros::ok())
		{
		std::cout << "Entering the main while loop" << std::endl;
		while (number_count >= 0.0 && number_count <= 1.0)
		{
		std::cout << "Entering the frist while loop" << std::endl;
		//Created a Float64 message
		std_msgs::Float64 msg;

		//Inserted data to message header
		msg.data = number_count;

		//Printing message data
		ROS_INFO("%d",msg.data);

		//Publishing the topic 
		pub->joint_publisher_1.publish(msg);
		pub->joint_publisher_2.publish(msg);
		pub->joint_publisher_3.publish(msg);
		pub->joint_publisher_4.publish(msg);
		pub->joint_publisher_5.publish(msg);
		pub->joint_publisher_6.publish(msg);
		pub->joint_publisher_7.publish(msg);

		//Spinning once for doing the  all operation once
		ros::spinOnce();

		//Sleeping for sometime
		loop_rate.sleep();

		// Writing to a file
		str = "number_count= " + std::to_string(number_count);
		log->writeToFile(str);
		//Incrementing the count
		number_count = number_count + 0.1;
		}

		while (number_count >= 0.0)
		{
		std::cout << "Entering the second while loop" << std::endl;
		//Created a Float64 message
		std_msgs::Float64 msg;

		//Inserted data to message header
		msg.data = number_count;

		//Printing message data
		ROS_INFO("%d",msg.data);
		if (number_count > 0.0 )
		{
		//Publishing the topic 
		pub->joint_publisher_1.publish(msg);
		pub->joint_publisher_2.publish(msg);
		pub->joint_publisher_3.publish(msg);
		pub->joint_publisher_4.publish(msg);
		pub->joint_publisher_5.publish(msg);
		pub->joint_publisher_6.publish(msg);
		pub->joint_publisher_7.publish(msg);

		//Spinning once for doing the  all operation once
		ros::spinOnce();

		//Sleeping for sometime
		loop_rate.sleep();

		//Incrementing the count
		number_count = number_count - 0.1;
		}
		else
			break;
	}
	number_count = 0.0;
	std::cout << "number_count= " << number_count << std::endl;
	}
}

void controller::setHomePoisition(const float &val)
{
		std::unique_ptr<publisher> pub(new publisher(1000));
		home_val.data = val;
		ros::Rate poll_rate(1000);
		while(pub->joint_publisher_1.getNumSubscribers() == 0 &&
			pub->joint_publisher_2.getNumSubscribers() == 0 && 
			pub->joint_publisher_3.getNumSubscribers() == 0 &&
			pub->joint_publisher_4.getNumSubscribers() == 0 &&
			pub->joint_publisher_5.getNumSubscribers() == 0 &&
			pub->joint_publisher_6.getNumSubscribers() == 0 &&
			pub->joint_publisher_7.getNumSubscribers() == 0)

			{
		 
			poll_rate.sleep();
			}
		//Publishing the topic 
		pub->joint_publisher_1.publish(home_val);
		pub->joint_publisher_2.publish(home_val);
		pub->joint_publisher_3.publish(home_val);
		pub->joint_publisher_4.publish(home_val);
		pub->joint_publisher_5.publish(home_val);
		pub->joint_publisher_6.publish(home_val);
		pub->joint_publisher_7.publish(home_val);
		
		//Spinning once for doing the  all operation once
		ros::spinOnce();
		
}