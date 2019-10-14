
#include <fstream>
#include <iostream>
class logger
{
public:
	logger() {};
	void writeToFile(std::string);

	~logger() {};

private:
	std::ofstream outfile;
};

void logger::writeToFile(std::string str)
{
		outfile.open("/home/karim-enien/CppND/Project_5_Building_A_ROS_NODE_With_Gazebo/src/robot_demo_movement_pkg/src/example.txt", std::ios::out | std::ios::app);

		if (outfile.is_open())
		{
			outfile << str << std::endl;
			//outfile << std::flush;
			outfile.close();
		}

	else
		std::cout << "String is empty!!!" << std::endl;
}