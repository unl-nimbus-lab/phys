#ifndef _KALMAN_H_
#define _KALMAN_H_
#include <Eigen/Dense>

void kalman_init();

Eigen::Matrix<float, 12, 1> kalman_propagate();

Eigen::Matrix<float, 12, 1> kalman_estimate_pos(Eigen::Matrix<float, 3, 1> z);

Eigen::Matrix<float, 12, 1> kalman_estimate_acc(Eigen::Matrix<float, 3, 1> z);

#endif