#include "ros/ros.h"
// #include "std_msgs/Int32.h"
#include "beginner_tutorials/Num.h"
#include "dxl_serial/dxl_serial.h"

DXL dxl("/dev/ttyUSB0",2000000); 

int main(int argc, char **argv)
{ 
  int serial_port;
  serial_port = dxl.Initialize();
  
  ROS_INFO("serial_port : %d", serial_port);
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<beginner_tutorials::Num>("chatter", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    beginner_tutorials::Num msg;
    msg.data = count;
    ROS_INFO("I speak : [%d]", msg.data);
    chatter_pub.publish(msg); 
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
