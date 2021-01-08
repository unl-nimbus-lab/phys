
#include "MatricesAndVectors.h"
#include "QuatRotEuler.h"
#include <math.h>
#include <Eigen/Dense>
using Eigen::Matrix;

//Rotation matrix around x
Matrix<float, 3, 3> Rotx(double theta){
	Matrix<float, 3, 3> R;

	R << 1,          0,           0,
	     0, cos(theta), -sin(theta),
	     0, sin(theta),  cos(theta);

	return R;
}

//Rotation matrix around y
Matrix<float, 3, 3> Roty(double theta){
	Matrix<float, 3, 3> R;

	R <<  cos(theta), 0, sin(theta),
	               0, 1,          0,
	     -sin(theta), 0, cos(theta);

	return R;
}

//Rotation matrix around z
Matrix<float, 3, 3> Rotz(double theta){
	Matrix<float, 3, 3> R;

	R <<  cos(theta), -sin(theta), 0,
	      sin(theta),  cos(theta), 0,
	               0,           0, 1;

	return R;
}

//Convert Euler Angles (Roll-pitch-yaw) to Rotation Matrix
Matrix<float, 3, 3> RPY2Rot(double roll, double pitch, double yaw){
	Matrix<float, 3, 3> Rx = Rotx(roll);
	Matrix<float, 3, 3> Ry = Roty(pitch);
	Matrix<float, 3, 3> Rz = Rotz(yaw);

	return Rz*Ry*Rx; //Rout = Rz*Ry*Rx
	
}

//Convert quaternion to rotation matrix 
Matrix<float, 3, 3> Quat2rot(Matrix<float, 4, 1> q){

	Matrix<float, 3, 3> R;
	double qw = q(0);
	double qx = q(1);
	double qy = q(2);
	double qz = q(3);

	R <<  1 - 2 * pow(qy, 2) - 2 * pow(qz, 2),               2 * qx*qy - 2 * qz*qw,               2 * qx*qz + 2 * qy*qw,
	                    2 * qx*qy + 2 * qz*qw, 1 - 2 * pow(qx, 2) - 2 * pow(qz, 2),               2 * qy*qz - 2 * qx*qw,
	                    2 * qx*qz - 2 * qy*qw,               2 * qy*qz + 2 * qx*qw, 1 - 2 * pow(qx, 2) - 2 * pow(qy, 2);

	return R;
}

//Convert rotation matrix to quaternion
//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
Matrix<float, 4, 1> Rot2quat(Matrix<float, 3, 3> M){
	double trace = M.trace();
	Matrix<float, 4, 1> q;
	if (trace > 0) {// M_EPSILON = 0
		double s = 0.5 / sqrt(trace + 1.0);
		q << 0.25 / s,
			(M(2,1) - M(1,2)) * s,
			(M(0,2) - M(2,0)) * s,
			(M(1,0) - M(0,1)) * s;
	}
	else {
		if (M(0,0) > M(1,1) && M(0,0) > M(2,2)) {
			double s = 2.0 * sqrt(1.0 + M(0,0) - M(1,1) - M(2,2));
			q << (M(2,1) - M(1,2)) / s,
				 0.25 * s,
				 (M(0,1) + M(1,0)) / s,
				 (M(0,2) + M(2,0)) / s;
		}
		else if (M(1,1) > M(2,2)) {
			double s = 2.0 * sqrt(1.0 + M(1,1) - M(0,0) - M(2,2));
			q << (M(0,2) - M(2,0)) / s;
			     (M(0,1) + M(1,0)) / s;
			     0.25 * s;
			     (M(1,2) + M(2,1)) / s;
		}
		else {
			double s = 2.0 * sqrt(1.0 + M(2,2) - M(0,0) - M(1,1));
			q << (M(1,0) - M(0,1)) / s;
			     (M(0,2) + M(2,0)) / s;
			     (M(1,2) + M(2,1)) / s;
			     0.25 * s;
		}
	}
        return q;
}

//Convert quaternion to Roll-Pitch-Yaw
Matrix<float, 3, 1> Quat2RPY(Matrix<float, 4, 1> q){

	Matrix<float, 3, 1> RPY;

	RPY << atan2(2*(q(0)*q(1) + q(2)*q(3)) , 1 - 2*(q(1)*q(1) + q(2)*q(2)) ),	//Roll
		   asin(2*(q(0)*q(2) - q(3)*q(1))),				//Pitch
		   atan2(2*(q(0)*q(3) + q(1)*q(2)),1 - 2*(q(2)*q(2) + q(3)*q(3)) );	//Yaw

	return RPY;
}

// Quaternion Multiplication
Matrix<float, 4, 1> QuaternionProduct(Matrix<float, 4, 1> q1, Matrix<float, 4, 1> q2){
	Matrix<float, 4, 4> H;
	H << q1(0), -q1(1), -q1(2), -q1(3),
	     q1(1),  q1(0), -q1(3),  q1(2),
	     q1(2),  q1(3),  q1(0), -q1(1),
	     q1(3), -q1(2),  q1(1),  q1(0);

	return H*q2;
}

//Triad Algorithm for Finding attitude from two vectors
// https://en.wikipedia.org/wiki/Triad_method
Matrix<float, 3, 3> Triad(Matrix<float, 3, 1> V1_body, Matrix<float, 3, 1> V2_body, Matrix<float, 3, 1> V1_inertial, Matrix<float, 3, 1> V2_inertial){
	Matrix<float, 3, 3> R, r;
	Matrix<float, 3, 1> r1, r2, r3, R1, R2, R3, V1_bodyNorm, V2_bodyNorm, V1_inertialNorm, V2_inertialNorm;

	//Normalize vectors and make them orthogonal
	// V1_bodyNorm = ScaleVec3(V1_body, 1.0 / p_normVec3(V1_body, 2));
	// V2_bodyNorm = ScaleVec3(V2_body, 1.0 / p_normVec3(V2_body, 2));
	// V1_inertialNorm = ScaleVec3(V1_inertial, 1.0 / p_normVec3(V1_inertial, 2));
	// V2_inertialNorm = ScaleVec3(V2_inertial, 1.0 / p_normVec3(V2_inertial, 2));

	V1_bodyNorm = V1_body * (1.0 / V1_body.norm());
	V2_bodyNorm = V2_body * (1.0 / V2_body.norm());
	V1_inertialNorm = V1_inertial * (1.0 / V1_inertial.norm());
	V2_inertialNorm = V2_inertial * (1.0 / V2_inertial.norm());

	//Set orthogonal vectors from body frame
	r1 = V1_bodyNorm; 
	r2 = V1_bodyNorm.cross(V2_bodyNorm) * (1.0/V1_bodyNorm.cross(V2_bodyNorm).norm()); //(V1 x V2)/norm(V1 x V2)
	r3 = r1.cross(r2);

	//Set orthogonal vectors from inertial frame
	R1 = V1_inertialNorm;
	R2 = V1_inertialNorm.cross(V2_inertialNorm) * (1.0/V1_inertialNorm.cross(V2_inertialNorm).norm()); //(V1 x V2)/norm(V1 x V2)
	R3= R1.cross(R2);

	//We need to find the rotation matrix that solves [R1:R2:R3]=Rot.[r1:r2:r3]
	// ==> Rot = [R1:R2:R3].[r1:r2:r3]'
	// Matrix<float, 3, 3> R = Concatenate3Vec3_2_Mat3x3(R1, R2, R3); //Creates matrix R = [R1:R2:R3]
	R << R1, R2, R3;
	//Matrix<float, 3, 3> r = Concatenate3Vec3_2_Mat3x3(r1, r2, r3); //Creates matrix r = [r1:r2:r3]
	r << r1, r2, r3;
	return R*r.transpose();
}

//Propagation quaternion for propagating quaternion with gyroscope data
//Equation 34 in https://users.aalto.fi/~ssarkka/pub/quat.pdf
Matrix<float, 4, 1> propagationQuat(Matrix<float, 3, 1> w, double dt){
	Matrix<float, 4, 1> dq; //delta quaternion
	double norm_w = w.norm();
	double angle = dt*norm_w / 2;
	double sin_angle = sin(angle);

	dq(0,0) = cos(angle);
	if (norm_w != 0){
		// dq(1,0) << w(0) * sin_angle / norm_w,
		// dq(2,0) = w(1) * sin_angle / norm_w;
		// dq(3,0) = w(2) * sin_angle / norm_w;
		dq.tail(3) = w * (sin_angle / norm_w);
	}
	else{
		dq.tail(3) << 0,
					  0,
					  0;
	}
	return dq;
}

//Propagate rotation matrix from initial point through angular velocity
//Equation 33 in https://users.aalto.fi/~ssarkka/pub/quat.pdf, but instead of
// using quaternion multiplication, we use rotation matrix multiplication
Matrix<float, 3, 3> propagateRotMat(Matrix<float, 3, 3> M0, Matrix<float, 3, 1> w, double dt){
	Matrix<float, 4, 1> dq = propagationQuat(w, dt);
	Matrix<float, 3, 3> dRot = Quat2rot(dq);

	return M0*dRot;
}