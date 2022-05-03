#include "ros/ros.h"
//위 라이브러리는 /opt/ros/melodic/include 폴더에서 찾아볼 수 있다.
#include "std_msgs/Int32.h"
#include <iostream> //inputoutstream

int main(int argc, char **argv){
	
  ros::init(argc, argv, "open_cr_module");
	
  ros::NodeHandle nh;
  ros::Publisher imu_data_pub = nh.advertise<std_msgs::Int32>("robotis/open_cr/imu", 10);
    
  ros::Rate loop_rate(10); //10Hz
  
  int number = 0;
  while(ros::ok()){    
    
    std_msgs::Int32 msg;

    msg.data = number;
        
    ROS_INFO("%d", msg.data);
        
    imu_data_pub.publish(msg);
        
    ros::spinOnce();
        
    loop_rate.sleep();
    number++;
  }
  
  return 0;
}
