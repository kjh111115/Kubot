#include "kinematics.h"



int main()
{
    std::cout << "hi" << std::endl;
    return 0;
}

Matrix4d jointToRotMatX(double roll)
{
    double q = roll;

    Matrix4d tmp_m;

    tmp_m(0,0) = 1; tmp_m(0,1) = 0;         tmp_m(0,2) = 0;         tmp_m(0,3) = 0;
    tmp_m(1,0) = 0; tmp_m(1,1) = cos(q);    tmp_m(1,2) = -sin(q);   tmp_m(1,3) = 0;
    tmp_m(2,0) = 0; tmp_m(2,1) = sin(q);    tmp_m(2,2) = cos(q);    tmp_m(2,3) = 0;
    tmp_m(3,0) = 0; tmp_m(3,1) = 0;         tmp_m(3,2) = 0;         tmp_m(3,3) = 1;

    Matrix4d RotX = tmp_m;

    return tmp_m;
}

Matrix4d jointToRotMatY(double pitch)
{
    double q = pitch;

    Matrix4d tmp_m;

    tmp_m(0,0) = cos(q);    tmp_m(0,1) = 0;     tmp_m(0,2) = sin(q);    tmp_m(0,3) = 0;
    tmp_m(1,0) = 0;         tmp_m(1,1) = 1;     tmp_m(1,2) = 0;         tmp_m(1,3) = 0;
    tmp_m(2,0) = -sin(q);   tmp_m(2,1) = 0;     tmp_m(2,2) = cos(q);    tmp_m(2,3) = 0;
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;     tmp_m(3,2) = 0;         tmp_m(3,3) = 1;

    Matrix4d RotY = tmp_m;
    
    return RotY;
}

Matrix4d jointToRotMatZ(double yaw)
{
    double q = yaw;

    Matrix4d tmp_m;

    tmp_m(0,0) = cos(q);    tmp_m(0,1) = -sin(q);   tmp_m(0,2) = 0; tmp_m(0,3) = 0;
    tmp_m(1,0) = sin(q);    tmp_m(1,1) =  cos(q);   tmp_m(1,2) = 0; tmp_m(1,3) = 0;
    tmp_m(2,0) = 0;         tmp_m(2,1) = 0;         tmp_m(2,2) = 1; tmp_m(2,3) = 0;
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;         tmp_m(3,2) = 0; tmp_m(3,3) = 1;

    Matrix4d RotZ = tmp_m;

    return RotZ;
}

Matrix4d jointToTransMat(Vector3d position)
{
    Matrix4d tmp_m;
    Vector3d tmp_v;

    tmp_v << 
        position(0), \
        position(1), \
        position(2);

    tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = tmp_v(0);
    tmp_m(1,0) = 0; tmp_m(1,1) = 1; tmp_m(1,2) = 0; tmp_m(1,3) = tmp_v(1);
    tmp_m(2,0) = 0; tmp_m(2,1) = 0; tmp_m(2,2) = 1; tmp_m(2,3) = tmp_v(2);
    tmp_m(3,0) = 0; tmp_m(3,1) = 0; tmp_m(3,2) = 0; tmp_m(3,3) = 1;


    return tmp_m;
}

VectorXd matToPos(Eigen::Matrix4d tfMat)
{
    Vector3d tmp_v;

    tmp_v(0) = tfMat(0,3);
    tmp_v(1) = tfMat(1,3);
    tmp_v(2) = tfMat(2,3);
    
    return tmp_v;
}

VectorXd matToEuler(Eigen::Matrix4d tfMat)
{
    Vector4d tmp_v;

    tmp_v(0) = atan2(tfMat(1,0), tfMat(0,0));
    tmp_v(1) = atan2(-tfMat(2,0), sqrt(pow(tfMat(2,1),2)+pow(tfMat(2,2),2)));
    tmp_v(2) = atan2(tfMat(2,1), tfMat(2,2));
    tmp_v(3) = 1;
    
    return tmp_v;
}

// How to use vector(6) in EigenLibrary?
VectorXd tfToSole(Eigen::VectorXd jointVar, Eigen::Vector3d linkParam1, Eigen::Vector3d linkParam2, Eigen::Vector3d linkParam3, Vector3d linkParam4)
{
    MatrixXd tmp_m(4,4);

    VectorXd q(6);
    Vector3d RP01, RP12, RP23, RP3E;

    /*  RP01[0] = relative position of Hip with respect to Base
        RP12[1] = relative position of Knee with respect to Hip
        RP23[2] = relative position of Ankle with respect to Knee
        RP3E[3] = relative position of Sole(End_effector) with respect to Ankle   */

    RP01 << linkParam1(0), linkParam1(1), linkParam1(2);
    RP12 << linkParam2(0), linkParam2(1), linkParam2(2);
    RP23 << linkParam3(0), linkParam3(1), linkParam3(2);
    RP3E << linkParam4(0), linkParam4(1), linkParam4(2);

    q <<jointVar(0), jointVar(1), jointVar(2), jointVar(3), jointVar(4), jointVar(5);

        jointVar(0), \
        jointVar(1), \
        jointVar(2), \
        jointVar(3), \
        jointVar(4), \
        jointVar(5);
    /*  q(0) = hip_yaw
        q(1) = hip_roll
        q(2) = hip_pitch
        q(3) = knee_pitch
        q(4) = ankle_pitch
        q(5) = ankle_roll   */

    tmp_m = jointToTransMat(RP01)*jointToRotMatZ(q(0))*jointToRotMatX(q(1))*jointToRotMatY(q(2))*   // Hip
            jointToTransMat(RP12)*jointToRotMatY(q(3))*                                             // Knee
            jointToTransMat(RP23)*jointToRotMatY(q(4))*jointToRotMatX(q(5))*                        // Pitch
            jointToTransMat(RP3E);                                                                  // Sole
    // base > hip_yaw > hip_roll > hip_pitch > knee_pitch > ankle_pitch > ankle_roll

    return tmp_m;
}

VectorXd tfToHand(Eigen::VectorXd jointVar, Eigen::Vector3d linkParam1, Eigen::Vector3d linkParam2, Eigen::Vector3d linkParam3)
{
    MatrixXd tmp_m(4,4);

    Vector3d q;
    Vector3d RP01, RP12, RP2E;

    RP01 << linkParam1(0), linkParam1(1), linkParam1(2);
    RP12 << linkParam2(0), linkParam2(1), linkParam2(2);
    RP2E << linkParam3(0), linkParam3(1), linkParam3(2);

    /*  RP01[0] = relative position of Arm with respect to Base
        RP12[1] = relative position of Elbow respect to Arm
        RP2E[2] = relative position of Hand with respect to Elbow   */

    q << 
        jointVar(0), \
        jointVar(1), \
        jointVar(2);
    /*  q(0) = sholder_pitch
        q(1) = shorlder_roll
        q(2) = elbow_pitch      */

    tmp_m = jointToTransMat(RP01)*jointToRotMatY(q(0))*jointToRotMatX(q(1))*                      // Shoulder
            jointToTransMat(RP12)*jointToRotMatY(q(3))*                                           // Elbow
            jointToTransMat(RP2E);                                       ;                        // Hand

    return tmp_m;
}