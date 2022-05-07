#ifndef KUBOT_H_
#define KUBOT_H_

// ROS library headers

#include "ros/ros.h"
#include "std_msgs/String.h"

// C library headers
#include <stdio.h>
#include <string.h>
#include <math.h>

// Eigen/RBDL library headers
#include <eigen3/Eigen/Dense>
#include <rbdl/rbdl.h>

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

// Kudos libraries
#include "dxl_serial/dxl_serial.h"
#include "kinematics.h"

int position(double time);

void Load();
void process();
void set_motor_id();

#endif // kubot