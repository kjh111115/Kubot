#include "ros/ros.h"
// #include "std_msgs/Int32.h"
#include "beginner_tutorials/Num.h"


void chatterCallback(const beginner_tutorials::Num::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
