#include "controller.h"
#include <string>


int readCmdLineArg()
{
  int param;
  ros::NodeHandle nh("~");
  nh.getParam("param", param);
  std::cout << "Got param= " << param << std::endl;
  if (param > 0 && param < 10)
    return param;
       
  else
    std::cout << "Enter Valid Parameter between 1 and 10 !! " << std::endl;

}
int main(int argc, char **argv)

{
	//Initializing ROS node with a name of demo_topic_publisher.
	ros::init(argc, argv,"movement_demo_topic_publisher");
	//int *arg = readCmdLineArgs(argc, argv);
	int loopRate = readCmdLineArg();
	//Intializing a controller object and setting the publishing rate to 1ms and the number count for the joint angles to 0 Radian.
	std::unique_ptr<controller> cont(new controller(loopRate,0));
	//Starting the main loop of the demo program.
	cont->mainLoop();
	
	return 0;
}


