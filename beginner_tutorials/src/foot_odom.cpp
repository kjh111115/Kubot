#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

void walkingStateCallback(const std_msgs::Int32::ConstPtr& msg2)
{
	if(msg2->data == 0)	
		ROS_INFO("stand : [%d]", msg2->data);
	else
		ROS_INFO("fallen : [%d]", msg2->data);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "foot_odom");
   
	ros::NodeHandle nh;
	ros::Subscriber stand_state_sub = nh.subscribe("robotis/walking/standup", 10, walkingStateCallback);
	
	ros::spin();
  return 0;
}
	

