#include "controller.h"


int main(int argc, char **argv)

{

	//Initializing ROS node with a name of demo_topic_publisher.
	ros::init(argc, argv,"movement_demo_topic_publisher");
	//Intializing a controller object and setting the publishing rate to 1ms and the number count for the joint angles to 0 Radian.
	std::unique_ptr<controller> cont(new controller(1,0));
	//Starting the main loop of the demo program.
	cont->mainLoop();
	
	return 0;
}


