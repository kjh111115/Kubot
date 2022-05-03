#include "kubot.h"

DXL dxl("/dev/ttyUSB0",2000000); 
Kinematics FK;

#define NSEC_PER_SEC 1000000000     // 10^9
#define EC_TIMEOUTMON 500

unsigned int cycle_ns = 1000000;    // 10^6ns = 1ms
unsigned int Run_time = 0;

// **********Xenomai time variables*********//

RT_TASK RT_task1;
//RT_TASK RT_task2;
//RT_TASK RT_task3;
// 객체 RT_task 생성 - thread 1개 돌리기

RTIME now1, previous1; // Ethercat time
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

    rt_task_create(&RT_task1, "serial_task", 0, 99, 0);
    rt_task_start(&RT_task1, &serial_task, NULL);

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
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 10);
    //어떤 thread에 주기 설정 (10ms)
    while (1)
    {
    // clock_t start = clock(); //
    // 리눅스로부터 시간을 받아와서 시간 반환
        rt_task_wait_period(NULL);
    // 이거는 제노마이 약간 다른 sleep 개념
   
        process();
    // clock_t end = clock(); //%%%%%%%%%%%%%%%%
    // printf("operating TiMe(ms): %lf\n",(double)(end - start)/CLOCKS_PER_SEC * 1000);
    }

    return;
}

void Load()
{
    dxl.motor_id_1 = 1;
    dxl.motor_id_2 = 2;

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
    dxl.sync_wirte(position(Run_time),position(Run_time));
//    dxl.buffer_data = dxl.read_buffer(dxl.serial_port);

    Run_time = Run_time + 10; // 
    // a = FK.jointToRotMatX(0.75);
    // std::cout << a << std::endl;
}

int position(int time)
{
    int period = 3000;
    int pos = int(2048*(1-cos(2*M_PI*time/period)));

    return pos;
}   