#include <iostream>
#include <math.h>
#include <Eigen/Dense>
using Eigen::Matrix;

float dt;
float vicon_R;
float accelerometer_R;
float sigma_J;
float sigma_bAcc;

Matrix<float, 12, 1> x_est;
Matrix<float, 12, 12> p_est;
Matrix<float, 12, 1> x_prd;
Matrix<float, 12, 12> p_prd;
Matrix<float, 12, 12> Q;           
Matrix<float, 3, 3> R_vicon;
Matrix<float, 3, 3> R_acc;
Matrix<float, 12, 12> A;
Matrix<float, 3, 12> H_pos;
Matrix<float, 3, 12> H_v;
Matrix<float, 3, 12> H_acc;  //Acceleration without bias
Matrix<float, 3, 12> H_accm; //Measured acceleration (bias added)
Matrix<float, 3, 3> S;
Matrix<float, 3, 12> B;
Matrix<float, 12, 3> K;
Matrix<float, 3, 1> y;
Matrix<float, 3, 1> v;
Matrix<float, 3, 1> a;
Matrix<float, 3, 1> bAcc;
Matrix<float, 12, 1> result;
Matrix<float, 12, 6> Gammak;
Matrix<float, 6, 6> Qk;
Matrix<float, 12, 12> I_12x12;
Matrix<float, 3, 3> I_3x3;
Matrix<float, 3, 3> Zero_3x3;


void kalman_init()
{

	dt = 0.010;             // 100 Hz
  sigma_J = 20;           // Jerk process noise
  sigma_bAcc = 0.01;      // Accelerometer bias process noise
	vicon_R = 0.0001;       // estimated vicon standard deviation
  accelerometer_R = 0.5; // estimated accelerometer standard deviation
  I_12x12 = Matrix<float, 12, 12>::Identity();
  I_3x3 = Matrix<float, 3, 3>::Identity();
  Zero_3x3 = Matrix<float, 3, 3>::Zero(3, 3);
  
  Gammak << I_3x3 * (pow(dt, 3)/6), Zero_3x3,
            I_3x3 * (pow(dt, 2)/2), Zero_3x3,
                        I_3x3 * dt, Zero_3x3,
                          Zero_3x3, I_3x3*dt;
  Qk << pow(sigma_J, 2)*I_3x3,                 Zero_3x3, 
                     Zero_3x3, pow(sigma_bAcc, 2)*I_3x3; //Process noise covariance matrix
  Q = Gammak*Qk*Gammak.transpose();

	R_vicon = pow(vicon_R, 2) * I_3x3;       // Vicon measurement covariance matrix
  R_acc = pow(accelerometer_R, 2) * I_3x3; // Accelerometer measurement covariance matrix

  A << I_3x3,    dt*I_3x3, (pow(dt,2)/2)*I_3x3, Zero_3x3,
       Zero_3x3,    I_3x3,            dt*I_3x3, Zero_3x3,
       Zero_3x3, Zero_3x3,               I_3x3, Zero_3x3,
       Zero_3x3, Zero_3x3,            Zero_3x3, I_3x3;

  //Initial estimate: zeros
  x_est = Matrix<float, 12, 1>::Zero();

  //Initial covariance: we assume we don't know initial position, but velocity and 
  // acceleration are close to zero. Also, we don't know the accelerometer bias
  p_est << 10*I_3x3,     Zero_3x3,     Zero_3x3,  Zero_3x3,
            Zero_3x3, 0.0001*I_3x3,     Zero_3x3,  Zero_3x3,
            Zero_3x3,     Zero_3x3, 0.0001*I_3x3,  Zero_3x3,
            Zero_3x3,     Zero_3x3,     Zero_3x3, 1000*I_3x3;

  H_pos  << I_3x3,    Zero_3x3, Zero_3x3, Zero_3x3;
  H_v    << Zero_3x3, I_3x3,    Zero_3x3, Zero_3x3;
  H_acc  << Zero_3x3, Zero_3x3,    I_3x3, Zero_3x3; //Acceleration we want to determine
  H_accm << Zero_3x3, Zero_3x3,    I_3x3,    I_3x3; //Measured acceleration (includes bias)


}

Matrix<float, 12, 1> kalman_propagate()
{
  // Predicting state and covariance

  x_prd = A * x_est;
  p_prd = A * p_est * A.transpose() + Q;

  y = H_pos * x_prd; // Position estimate
  v = H_v * x_prd;   // Velocity estimate
  a = H_acc * x_prd; // Velocity estimate
  bAcc = (H_accm - H_acc)*x_prd; //Bias estimate

  result << y,
            v,
            a,
            bAcc;

  x_est = x_prd;
  p_est = p_prd;

  return result;

}

Matrix<float, 12, 1> kalman_estimate_pos(Matrix<float, 3, 1> z)
{

// Estimating state and covariance

  S = H_pos * p_prd.transpose() * H_pos.transpose() + R_vicon;
  B = H_pos * p_prd.transpose();
  K = (S.inverse() * B).transpose();

  x_est = x_prd + K * (z - H_pos * x_prd);
  p_est = (I_12x12 - K * H_pos) * p_prd * (I_12x12 - K * H_pos).transpose() + K * R_vicon * K.transpose();   // Joseph form 

// Computing measurements
  y = H_pos * x_est;   // Position estimate
  v = H_v * x_est; // Velocity estimate
  a = H_acc * x_est; // Acc estimate
  bAcc = (H_accm - H_acc)*x_prd; //Bias estimate

  result << y,
            v,
            a,
            bAcc;

  //These are necessary if the acceleration is updated
  x_prd = x_est;
  p_prd = p_est;

  return result;
}


Matrix<float, 12, 1> kalman_estimate_acc(Matrix<float, 3, 1> z)
{

// Estimating state and covariance

  S = H_accm * p_prd.transpose() * H_accm.transpose() + R_acc;
  B = H_accm * p_prd.transpose();
  K = (S.inverse() * B).transpose();

  x_est = x_prd + K * (z - H_accm * x_prd);
  p_est = (I_12x12 - K * H_accm) * p_prd * (I_12x12 - K * H_accm).transpose() + K * R_acc * K.transpose();   // Joseph form 

// Computing measurements
  y = H_pos * x_est;   // Position estimate
  v = H_v * x_est; // Velocity estimate
  a = H_acc * x_est;
  bAcc = (H_accm - H_acc)*x_prd; //Bias estimate

  result << y,
            v,
            a,
            bAcc;

  // std::cout << x_est << std::endl << std::endl;
  // std::cout << p_est << std::endl << std::endl;

  return result;
}


// int main()
// {

// 	kalman_init();

// 	MatrixXd z(3,1);   // measurement vector

// 	z << 1,
// 	     1,
// 	     1;

// 	std::cout << kalman(z) << std::endl;

// }