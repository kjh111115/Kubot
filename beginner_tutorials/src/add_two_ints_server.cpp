#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h" 
// AddTwoInts.h : ~/catkin_ws/devel/include/beginner_tutorials/.

#define PLUS 1
#define MINUS 2
#define MULTIPLICATION 3
#define DVISITION 4

int g_operator = PLUS;

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
         // request : 요청  -  int64 a, int64 b
         // response : 응답 - int64 result
{
  switch (g_operator)
  {
  case PLUS:
    res.result = req.a + req.b;
    break;
  
  case MINUS:
    res.result = req.a - req.b;
    break;

  case MULTIPLICATION:
    res.result = req.a * req.b;
    break;
  
  case DVISITION:
    if(req.b == 0){
      res.result = 0;
      break;
    }
    else{
      res.result = req.a / req.b;
      break;
    }
  default:
    res.result = req.a + req.b;
    break;
  }

  
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b); //ros printf
  ROS_INFO("sending back response: [%ld]", (long int)res.result);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle nh;

  nh.setParam("calculation_method", PLUS); // calculation_method 라는 이름의 매개변수의 초기값을 PLUS로 설정

  ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  // ros::spin();

  ros::Rate loop_rate(10);
  // while(1) 이걸로 하면 ctrl+c 로 터미널이 안꺼진다.
  while(ros::ok())
  {
    nh.getParam("calculation_method", g_operator); 
    // calculation_method 라는 이름의 매개변수를 불러와서 g_operatior의 값으로 설정
    // 이에 따라 g_operator는 0.1초(10hz) 마다 매개변수의 값을 확인하여 서비스 요청으로 받은 값을 사칙연산 중 어떤 계산을 하여 처리할지 결정
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}