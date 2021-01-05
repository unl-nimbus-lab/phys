#include <Eigen/Dense>
using Eigen::Matrix;
//Functions

//Attitude error: e_r = 0.5*invSkew(Rref'*Rbw - Rbw'*Rref)
Matrix<float, 3, 1> AttitudeErrorVector(Matrix<float, 3, 3> Rbw, Matrix<float, 3, 3> Rdes);

//Attitude control inputs: u_att = -Kr*e_r - Kw*e_w
Matrix<float, 3, 1> AttitudeControlInputs(Matrix<float, 3, 3> Kr, Matrix<float, 3, 3> Kw, Matrix<float, 3, 1> e_r, Matrix<float, 3, 1> e_w);


/*Return inputs to quadrotor for attitude control
q[4];		//Attitude quaternion
Rdes;		//Desired rotation matrix
w_bw[3];	//Angular velocity of the system
wDes[3];	//Desired angular velocity
u1;			//Thrust 
This algorithm was extracted from Mellinger, 2011: Minimum Snap Trajectory Generation and Control for Quadrotors*/
Matrix<float, 4, 1> attitudeControl(Matrix<float, 4, 1> q, Matrix<float, 3, 3> Rdes, Matrix<float, 3, 1> w_bw, Matrix<float, 3, 1> wDes, float u1);

/* Converts desired thrusts per motor into PWM values.
Solution to quadratic equation KT_a.x^2 + KT_b.x - thrust = 0 */
Matrix<float, 4, 1> Thrusts2PWM(Matrix<float, 4, 1> thrusts);

/* Convert inputs u = [Thrust, torque_roll, torque_pitch, torque_yaw] into pwm values (T coordinates)
T coordinates assumes: u = [1    1   1    1  ] [PWM1(T1)]
						   [0   -L   0    L  ] [PWM2(T2)]
						   [-L   0   L    0  ] [PWM3(T3)]
						   [Ctm -Ctm Ctm -Ctm] [PWM4(T4)]
Ctm = ratio between Moment and Thrust per propeller */
Matrix<float, 4, 1> u2pwmTshape(Matrix<float, 4, 1> u);

/* Convert inputs u = [Thrust, torque_roll, torque_pitch, torque_yaw] into pwm values (X coordinates)
X coordinates assumes: u =[1    0      0       0 ] [1    1   1    1  ] [PWM1(T1)]
						  [0    cos(o) -sin(o) 0 ] [0   -L   0    L  ] [PWM2(T2)]
						  [0    sin(o) cos(o)  0 ] [-L   0   L    0  ] [PWM3(T3)]
						  [0    0      0       1 ] [Ctm -Ctm Ctm -Ctm] [PWM4(T4)]    
Ctm = ratio between Moment and Thrust per propeller */
Matrix<float, 4, 1> u2pwmXshape(Matrix<float, 4, 1> u);
