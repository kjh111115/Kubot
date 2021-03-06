#ifndef Kinematics_H_
#define Kinematics_H_

#include "ros/ros.h"
#include "std_msgs/String.h"

// C library headers
#include <stdio.h>
#include <string.h>
#include <math.h>

// Eigen library headers
#include <eigen3/Eigen/Dense>

// typedef unsigned char BYTE;
// typedef ei;;4D 4X4MAT

using Eigen::Matrix4d;
using Eigen::MatrixXd;

using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector3d;

class Kinematics
{
public:
    Matrix4d jointToRotMatX(double roll);
    Matrix4d jointToRotMatY(double pitch);
    Matrix4d jointToRotMatZ(double yaw);
    Matrix4d jointToTransMat(Vector3d position);
    VectorXd matToPos(Matrix4d tfMat);
    VectorXd matToEuler(Matrix4d tfMat);
    Matrix4d tfToSole(VectorXd jointVar, Vector3d linkParam1, Vector3d linkParam2, Vector3d linkParam3, Vector3d linkParam4);
    Matrix4d tfToHand(VectorXd jointVar, Vector3d linkParam1, Vector3d linkParam2, Vector3d linkParam3);
};

#endif // Kinematics