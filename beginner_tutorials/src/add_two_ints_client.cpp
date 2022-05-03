#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib> //atoll지원

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;

  // 서비스 요청 값으로 노드가 실행될 때 입력으로 사용된 매개변수를 각각의 a, b 에 저장한다.
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  
  if (client.call(srv)) 
// If the service call succeeded, call() will return true
// and the value in srv.response will be valid. 
  {
    ROS_INFO("result: %ld", (long int)srv.response.result); //%ld : long int
  }
  else
// If the call did not succeed, call() will return false 
// and the value in srv.response will be invalid.
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;

  // there is no  ' ros::spin() ', so it closes when run only 1 time

}
