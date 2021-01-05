//
//#define NR_END 1
///*
// *  MatrixMath.h Library for Matrix Math
// *
// *  Created by Charlie Matlack on 12/18/10.
// *  Modified from code by RobH45345 on Arduino Forums, algorithm from 
// *  NUMERICAL RECIPES: The Art of Scientific Computing.
// */
//#define MatrixMath_h
//
//#if defined(ARDUINO) && ARDUINO >= 100
//#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif
//
//class MatrixMath
//{
//public:
//	//MatrixMath();
//	void Print(float* A, int m, int n, String label);
//	void Copy(float* A, int n, int m, float* B);
//	void Multiply(float* A, float* B, int m, int p, int n, float* C);
//	void Add(float* A, float* B, int m, int n, float* C);
//	void Subtract(float* A, float* B, int m, int n, float* C);
//	void Transpose(float* A, int m, int n, float* C);
//	void Scale(float* A, int m, int n, float k);
//	int Invert(float* A, int n);
//};
//
//
//MatrixMath Matrix;			// Pre-instantiate
//
//
//#define NUMX 6
//#define NUMU 6
//#define NUMV 2
//#define TUA_X 0.00001
//#define TUA_Y 0.00001
//#define MASS 1.6f
//#define k_v 0.5f/MASS
//float X[NUMX];
//float H[NUMV][NUMX];
//float A[NUMX][NUMX];
//float R[NUMV][NUMV];
////φ −Roll angle in current orientation estimate
////θ −Pitch angle in current orientation estimate
////βx −Bias in X axis gyroscope
////βy −Bias in Y axis gyroscope
////b
////vx −X velocity component of quadrotor in body frame
////b
////vy −Y velocity component of quadrotor in body frame
//
//
//// // gyro noise variance (rad/s)^2                 50e-4f;        
//// // accelerometer noise variance (m/s^2)^2        0.00001f;      
//// // gyro bias random walk variance (rad/s^2)^2    2e-8f;         
//
//
//
//
//
//
//
//
//// swarp x,y gyro on update plz
//
//
//void StateEq(float X[NUMX], float U[NUMU], float Xdot[NUMX])
//{        //Xdot = Output of stateeq    angle_dot[2]  bias_dot[2]  vel_dot[2]
//        //f เล็ก
//        //U control parameter gyro[0-2] bias_z[3]
//        //X state variable angle [0-2] gyro_bias[3-4] vel[5 - 6]  //my quad is Roll = gyroX axis 
//	float roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate,gyro_bias[3],vel[2];
//        
//        //sign here       กำหนดโดย thesis pap116
//        roll           = X[0];
//        pitch          = X[1];
//        yaw            = X[2];
//        gyro_bias[0]   = X[3];
//        gyro_bias[1]   = X[4];
//        
//        pitch_rate      = U[0];  //gyro pitch axis
//        roll_rate       = U[1];  //gyro roll  axis
//        yaw_rate        = U[2];
//        gyro_bias[2]    = U[3];
//        
//        
//        //angledot
//        Xdot[0] = (pitch_rate-gyro_bias[0])+tan(roll)*sin_pitch*(roll_rate-gyro_bias[1])+tan(roll)*cos_pitch*(yaw_rate-gyro_bias[2]);
//        Xdot[1] = cos_pitch*(roll_rate-gyro_bias[1])-sin_pitch*(yaw_rate-gyro_bias[2]);  //roll dot use gyro x
//        
//        //biasdot
//        Xdot[2] = -gyro_bias[0]/TUA_X; 
//        Xdot[3] = -gyro_bias[1]/TUA_Y;
//        
//        Xdot[4] = -9.80655*sin(roll)-k_v*vel[0];                 //x direction accel
//        Xdot[5] = 9.80655*cos_roll*sin_pitch-k_v*vel[1];         //y direction accel
//	
//}
//
//
//
//
////void INSSetState(float angle[2], float gyro_bias[2],float vel[2]) {
////        X[0] = angle[0]; //ahrs_r
////	X[1] = angle[1]; //ahrs_p
////	X[2] = gyro_bias[0]; //bias x
////	X[3] = gyro_bias[1]; //bias y
////	X[4] = vel[0];    //vel x
////	X[5] = vel[1];   //vel y
////}
//
//void RungeKutta(float X[NUMX], float U[NUMU], float dT)
//{
//
//	float dT2 =
//	    dT / 2.0f, K1[NUMX], K2[NUMX], K3[NUMX], K4[NUMX], Xlast[NUMX];
//	uint8_t i;
//
//	for (i = 0; i < NUMX; i++)
//		Xlast[i] = X[i];	// make a working copy
//
//	StateEq(X, U, K1);	// k1 = f(x,u)
//	for (i = 0; i < NUMX; i++)
//		X[i] = Xlast[i] + dT2 * K1[i];
//	StateEq(X, U, K2);	// k2 = f(x+0.5*dT*k1,u)
//	for (i = 0; i < NUMX; i++)
//		X[i] = Xlast[i] + dT2 * K2[i];
//	StateEq(X, U, K3);	// k3 = f(x+0.5*dT*k2,u)
//	for (i = 0; i < NUMX; i++)
//		X[i] = Xlast[i] + dT * K3[i];
//	StateEq(X, U, K4);	// k4 = f(x+dT*k3,u)
//
//	// Xnew  = X + dT*(k1+2*k2+2*k3+k4)/6
//	for (i = 0; i < NUMX; i++)
//		X[i] =
//		    Xlast[i] + dT * (K1[i] + 2.0f * K2[i] + 2.0f * K3[i] +
//				     K4[i]) / 6.0f;
//}
//
//void LinearizeH()
//{
//    H[0][0]=0; H[0][1]=0; H[0][2]=0;  H[0][3]=0; H[0][4]=-k_v; H[0][5]=0;
//    H[1][0]=0; H[1][1]=0; H[1][2]=0;  H[1][3]=0; H[1][4]=0;    H[1][5]=-k_v;
//}
//void R_update() {
//  R[0][0]=0.00001f;
//  R[1][1]=0.00001f;
//}
//void LinearizeQ(float dt)
//{
//  Q[0][0]=(50e-4f)*dt;
//  Q[1][1]=(50e-4f)*dt;
//  Q[2][2]=(2e-8f)*dt;
//  Q[3][3]=(2e-8f)*dt;
//  Q[4][4]=0.01f*dt;  //tune this
//  Q[5][5]=0.01f*dt;
//}
//void LinearizeA(float X[NUMX], float U[NUMU], float A[NUMX][NUMX],float dt)
//{
//        float roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate,gyro_bias[3],vel[2];
//        
//        //sign here       กำหนดโดย thesis pap116
//        roll           = X[0];
//        pitch          = X[1];
//        yaw            = X[2];
//        gyro_bias[0]   = X[3];
//        gyro_bias[1]   = X[4];
//        
//        pitch_rate      = U[0];  //gyro pitch axis
//        roll_rate       = U[1];  //gyro roll  axis
//        yaw_rate        = U[2];
//        gyro_bias[2]    = U[3];
//        
//    A[0][2] = -1*dt;
//    A[0][4]=0;
//    A[0][5]=0;
//    A[1][1]=0;
//    A[1][2]=0;
//    A[1][4]=0;
//    A[1][5]=0;
//    A[2][0]=0;
//    A[2][1]=0;
//    A[2][3]=0;
//    A[2][4]=0;
//    A[2][5]=0;
//    A[3][0]=0;
//    A[3][1]=0;
//    A[3][2]=0;
//    A[3][4]=0;
//    A[3][5]=0;
//    A[4][0]=0;
//    A[4][2]=0;
//    A[4][3]=0;
//    A[4][5]=0;
//    A[5][2]=0;
//    A[5][3]=0;
//    A[5][4]=0;
//    
//    
//    A[0][3] = -tan(roll)*sin_pitch*dt;
//    A[1][3] = -cos_pitch*dt;
//    A[4][1] = -ONE_G*cos_roll*dt;
//    A[5][0] = ONE_G*cos_roll*cos_pitch*dt;
//    A[5][1] = -ONE_G*sin_roll*sin_pitch*dt;
//    
//    A[2][2] =(-1/TUA_X)*dt+1;
//    A[3][3] =(-1/TUA_Y)*dt+1;
//    A[4][4] =(-k_v)*dt+1;
//    A[5][5] =(-k_v)*dt+1;
//    
//    
//    float a11,a12,a21;
//    A[0][0]= a11 = (tan(roll)*cos_pitch*(roll_rate-gyro_bias[1])-tan(roll)*sin_pitch*(yaw_rate-gyro_bias[2]))*dt+1;
//    A[0][1]= a12 = (sin_pitch*(roll_rate-gyro_bias[1])+cos_pitch*(yaw_rate-gyro_bias[2]))*dt/(cos_roll*cos_roll);
//    A[1][0]= a21 = (-sin_pitch*(roll_rate-gyro_bias[1])-cos_pitch*(yaw_rate-gyro_bias[2]))*dt;
//    
//    
//}
//
//
//
//
//
//
//
//
//
//
////******************************************************************************************
//// Matrix Printing Routine
//// Uses tabs to separate numbers under assumption printed float width won't cause problems
//void MatrixMath::Print(float* A, int m, int n, String label){
//	// A = input matrix (m x n)
//	int i,j;
//	Serial.println();
//	Serial.println(label);
//	for (i=0; i<m; i++){
//		for (j=0;j<n;j++){
//			Serial.print(A[n*i+j]);
//			Serial.print("\t");
//		}
//		Serial.println();
//	}
//}
//
//void MatrixMath::Copy(float* A, int n, int m, float* B)
//{
//	int i, j, k;
//	for (i=0;i<m;i++)
//		for(j=0;j<n;j++)
//		{
//			B[n*i+j] = A[n*i+j];
//		}
//}
//
////Matrix Multiplication Routine
//// C = A*B
//void MatrixMath::Multiply(float* A, float* B, int m, int p, int n, float* C)
//{
//	// A = input matrix (m x p)
//	// B = input matrix (p x n)
//	// m = number of rows in A
//	// p = number of columns in A = number of rows in B
//	// n = number of columns in B
//	// C = output matrix = A*B (m x n)
//	int i, j, k;
//	for (i=0;i<m;i++)
//		for(j=0;j<n;j++)
//		{
//			C[n*i+j]=0;
//			for (k=0;k<p;k++)
//				C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
//		}
//}
//
//
////Matrix Addition Routine
//void MatrixMath::Add(float* A, float* B, int m, int n, float* C)
//{
//	// A = input matrix (m x n)
//	// B = input matrix (m x n)
//	// m = number of rows in A = number of rows in B
//	// n = number of columns in A = number of columns in B
//	// C = output matrix = A+B (m x n)
//	int i, j;
//	for (i=0;i<m;i++)
//		for(j=0;j<n;j++)
//			C[n*i+j]=A[n*i+j]+B[n*i+j];
//}
//
//
////Matrix Subtraction Routine
//void MatrixMath::Subtract(float* A, float* B, int m, int n, float* C)
//{
//	// A = input matrix (m x n)
//	// B = input matrix (m x n)
//	// m = number of rows in A = number of rows in B
//	// n = number of columns in A = number of columns in B
//	// C = output matrix = A-B (m x n)
//	int i, j;
//	for (i=0;i<m;i++)
//		for(j=0;j<n;j++)
//			C[n*i+j]=A[n*i+j]-B[n*i+j];
//}
//
//
////Matrix Transpose Routine
//void MatrixMath::Transpose(float* A, int m, int n, float* C)
//{
//	// A = input matrix (m x n)
//	// m = number of rows in A
//	// n = number of columns in A
//	// C = output matrix = the transpose of A (n x m)
//	int i, j;
//	for (i=0;i<m;i++)
//		for(j=0;j<n;j++)
//			C[m*j+i]=A[n*i+j];
//}
//
//void MatrixMath::Scale(float* A, int m, int n, float k)
//{
//	for (int i=0; i<m; i++)
//		for (int j=0; j<n; j++)
//			A[n*i+j] = A[n*i+j]*k;
//}
//
//
////Matrix Inversion Routine
//// * This function inverts a matrix based on the Gauss Jordan method.
//// * Specifically, it uses partial pivoting to improve numeric stability.
//// * The algorithm is drawn from those presented in 
////	 NUMERICAL RECIPES: The Art of Scientific Computing.
//// * The function returns 1 on success, 0 on failure.
//// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
//int MatrixMath::Invert(float* A, int n)
//{
//	// A = input matrix AND result matrix
//	// n = number of rows = number of columns in A (n x n)
//	int pivrow;		// keeps track of current pivot row
//	int k,i,j;		// k: overall index along diagonal; i: row index; j: col index
//	int pivrows[n]; // keeps track of rows swaps to undo at end
//	float tmp;		// used for finding max value and making column swaps
//
//	for (k = 0; k < n; k++)
//	{
//		// find pivot row, the row with biggest entry in current column
//		tmp = 0;
//		for (i = k; i < n; i++)
//		{
//			if (abs(A[i*n+k]) >= tmp)	// 'Avoid using other functions inside abs()?'
//			{
//				tmp = abs(A[i*n+k]);
//				pivrow = i;
//			}
//		}
//
//		// check for singular matrix
//		if (A[pivrow*n+k] == 0.0f)
//		{
//			Serial.println("Inversion failed due to singular matrix");
//			return 0;
//		}
//
//		// Execute pivot (row swap) if needed
//		if (pivrow != k)
//		{
//			// swap row k with pivrow
//			for (j = 0; j < n; j++)
//			{
//				tmp = A[k*n+j];
//				A[k*n+j] = A[pivrow*n+j];
//				A[pivrow*n+j] = tmp;
//			}
//		}
//		pivrows[k] = pivrow;	// record row swap (even if no swap happened)
//
//		tmp = 1.0f/A[k*n+k];	// invert pivot element
//		A[k*n+k] = 1.0f;		// This element of input matrix becomes result matrix
//
//		// Perform row reduction (divide every element by pivot)
//		for (j = 0; j < n; j++)
//		{
//			A[k*n+j] = A[k*n+j]*tmp;
//		}
//
//		// Now eliminate all other entries in this column
//		for (i = 0; i < n; i++)
//		{
//			if (i != k)
//			{
//				tmp = A[i*n+k];
//				A[i*n+k] = 0.0f;  // The other place where in matrix becomes result mat
//				for (j = 0; j < n; j++)
//				{
//					A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
//				}
//			}
//		}
//	}
//
//	// Done, now need to undo pivot row swaps by doing column swaps in reverse order
//	for (k = n-1; k >= 0; k--)
//	{
//		if (pivrows[k] != k)
//		{
//			for (i = 0; i < n; i++)
//			{
//				tmp = A[i*n+k];
//				A[i*n+k] = A[i*n+pivrows[k]];
//				A[i*n+pivrows[k]] = tmp;
//			}
//		}
//	}
//	return 1;
//}
