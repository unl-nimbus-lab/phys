#ifndef FSW_CONTROL_QUATROTEULER_H
#define FSW_CONTROL_QUATROTEULER_H

#include "MatricesAndVectors.h"
#include <Eigen/Dense>
using Eigen::Matrix;

//Rotation matrix around x
Matrix<float, 3, 3> Rotx(double theta);

//Rotation matrix around y
Matrix<float, 3, 3> Roty(double theta);

//Rotation matrix around z
Matrix<float, 3, 3> Rotz(double theta);

//Convert Euler Angles (Roll-pitch-yaw) to Rotation Matrix
Matrix<float, 3, 3> RPY2Rot(double roll, double pitch, double yaw);

//Convert quaternion to rotation matrix 
Matrix<float, 3, 3> Quat2rot(Matrix<float, 4, 1> q);

//Convert rotation matrix to quaternion
Matrix<float, 4, 1> Rot2quat(Matrix<float, 3, 3> M);

//Convert quaternion to Roll-Pitch-Yaw
Matrix<float, 3, 1> Quat2RPY(Matrix<float, 4, 1> q);

// Quaternion Multiplication
Matrix<float, 4, 1> QuaternionProduct(Matrix<float, 4, 1> q1, Matrix<float, 4, 1> q2);

//Triad Algorithm for Finding attitude from two vectors
Matrix<float, 3, 3> Triad(Matrix<float, 3, 1> V1_body,Matrix<float, 3, 1> V2_body,Matrix<float, 3, 1> V1_inertial,Matrix<float, 3, 1> V2_inertial);

//Propagation quaternion for propagating quaternion with gyroscope data
//Equation 34 in https://users.aalto.fi/~ssarkka/pub/quat.pdf
Matrix<float, 4, 1> propagationQuat(Matrix<float, 3, 1> w, double dt);

//Propagate rotation matrix from initial point through angular velocity
//Equation 33 in https://users.aalto.fi/~ssarkka/pub/quat.pdf, but instead of
// using quaternion multiplication, we use rotation matrix multiplication
Matrix<float, 3, 3> propagateRotMat(Matrix<float, 3, 3> M0, Matrix<float, 3, 1> w, double dt);

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
