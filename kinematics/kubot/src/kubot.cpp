#include "kubot.h"

DXL dxl("/dev/ttyUSB0",2000000); 
Kinematics FK;

using namespace std;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#define NSEC_PER_SEC 1000000000     // 10^9
#define EC_TIMEOUTMON 500

unsigned int cycle_ns = 1000000;    // 10^6ns = 1ms
double Run_time = 0;

// **********Xenomai time variables*********//

//RT_TASK RT_task1;
//RT_TASK RT_task2;
//RT_TASK RT_task3;
// 객체 RT_task 생성 - thread 1개 돌리기

//RTIME now1, previous1; // Ethercat time
// 시간 측정하는 변수 - 제노마이 라이브러리에서 사용하는 시간 측정을 위한 변수
// 리
//RTIME now2, previous2; // Thread 1 cycle time
//RTIME now3, previous3; // Thread 2 cycle time

void serial_task(void* arg);
// thread를 사용해서 serial 통신 /모터 제어 등등할 함수이름

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kubot_node");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);
// Hz ->main 함수가 종료되면 안돼
// 그래서 무한 루프로 돌아
// while 한번 돌때마다 시간 쉬는거 없이 최대속도로 돌게 되는데

//    rt_task_create(&RT_task1, "serial_task", 0, 99, 0);
//    rt_task_start(&RT_task1, &serial_task, NULL);

// 병렬적으로 새로운 thread가 실행
// 기존 main 함수와 별개로 독립적으로 실행

    Load();
    
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "i said hello";
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void serial_task(void* arg)
{
//   rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 10);

//    previous1 = rt_timer_read();
    while (1)
    {
//        rt_task_wait_period(NULL);
   
        process();

//        now1 = rt_timer_read();
//        Run_time += (long)(now1-previous1)/1000;
//        cout << "Total_time : " << int(Run_time/cycle_ns) << "s  dt : " << (long)(now1-previous1)/1000 << "ns" << endl;

//        previous1 = now1;
    }
    return;
}   

void Load()
{
    set_motor_id();

    dxl.serial_port = dxl.Initialize();

    dxl.Torque_On(1);
    dxl.Torque_On(2);
    // dxl.Torque_On(3);
    // dxl.Torque_On(4);
    // dxl.Torque_On(5);
    // dxl.Torque_On(6);
}

void process()
{
    dxl.sync_write(position(Run_time/1000),position(Run_time/1000));
    dxl.buffer_data = dxl.read_buffer(dxl.serial_port);
}

int position(double time)
{
    int period = 3000;  // 3s
    int pos = int(2048*(1-cos(2*M_PI*time/period)));

    return pos;
}   

void set_motor_id()
{ 
  ROS_INFO("setting dynamixel ID...");

  dxl.motor_num = 2;
  for(int i=0 ; i< dxl.motor_num ; i++){
    dxl.motor[i] = i+1;
    ROS_INFO("dxl_motor[i] = i", dxl.motor_num, dxl.motor_num + 1);
  }
}