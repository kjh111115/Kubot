#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <iostream>


int stand_state;

void numberCallback(const std_msgs::Int32::ConstPtr& msg){
	ROS_INFO("Received: [%d]", msg->data);

	if(msg->data %2 == 0)
		stand_state= 0;
	else
		stand_state = 1;
}

void printfunction()
{
	ROS_INFO("I published");
}

int main(int argc, char **argv){

	ros::init(argc, argv, "soccer_demo");
   
	ros::NodeHandle nh;

	ros::Subscriber imu_data_sub = nh.subscribe("robotis/open_cr/imu", 10, numberCallback);
	
	ros::Publisher stand_state_pub = nh.advertise<std_msgs::Int32>("robotis/walking/standup", 10);	
	
	ros::Rate loop_rate(10);

	while(ros::ok()){
		std_msgs::Int32 msg2;
 	
		msg2.data = stand_state;
		stand_state_pub.publish(msg2);

		printfunction();
		
		ros::spinOnce();
    	loop_rate.sleep();
	}
	ros::spin();
   
	return 0;
}

