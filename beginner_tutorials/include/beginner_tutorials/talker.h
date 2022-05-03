#ifndef TALKER_H
#define TALKER_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


class talkerclass{
public:
	talkerclass();
	~talkerclass();
	double num;
	void printfunction(double num);
protected:
	ros::NodeHandle nh;
};

#endif /* TALKER_H */
