#ifndef KUBOT_H_
#define KUBOT_H_

// ROS library headers

#include "ros/ros.h"
#include "std_msgs/String.h"

// C library headers
#include <stdio.h>
#include <string.h>
#include <math.h>

// Eigen library headers
#include <eigen3/Eigen/Dense>

// Basic libraries
#include <sys/mman.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h>
#include <inttypes.h>

// Xenomai libraries

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>

#include "dxl_serial/dxl_serial.h"
#include "kinematics.h"

int position(int time);

void Load();
void process();

#endif // kubot