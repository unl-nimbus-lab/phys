

#include "MatricesAndVectors.h"
#include <stdio.h>		/* printf */
#include <math.h>		/* pow, sqrt */
#include <Eigen/Dense>
using Eigen::Matrix;
/* Create a 3x3 diagonal matrix from a 3x1 vector */
// Matrix3f Diag3(double vec[3]){
// 	Mat3x3 output;
// 	for (int i = 0; i < 3; i++){
// 		for (int j = 0; j < 3; j++){
// 			if (i == j)
// 				output.M[i][j] = vec[i];
// 			else
// 				output.M[i][j] = 0;
// 		}
// 	}

// 	return output;
// }

// Multiply 3x3 matrices: M_out = M1*M2
// Mat3x3 MultiplyMat3x3(Mat3x3 M1, Mat3x3 M2){
// 	Mat3x3 M_out;

// 	for (int i = 0; i < 3; i++){
// 		for (int j = 0; j < 3; j++){
// 			M_out.M[i][j] = 0;
// 			for (int k = 0; k < 3; k++){
// 				M_out.M[i][j] += M1.M[i][k] * M2.M[k][j];
// 			}
// 		}
// 	}

// 	return M_out;
// }

// Sum 3x3 matrices: M_out = M1 + M2
// Mat3x3 AddMat3x3(Mat3x3 M1, Mat3x3 M2){
// 	Mat3x3 M_out;

// 	for (int i = 0; i < 3; i++){
// 		for (int j = 0; j < 3; j++){
// 			M_out.M[i][j] = M1.M[i][j] + M2.M[i][j];
// 		}
// 	}

// 	return M_out;
// }


// // Subtract 3x3 matrices: M_out = M1 - M2
// Mat3x3 SubtractMat3x3(Mat3x3 M1, Mat3x3 M2){
// 	Mat3x3 M_out;

// 	for (int i = 0; i < 3; i++){
// 		for (int j = 0; j < 3; j++){
// 			M_out.M[i][j] = M1.M[i][j] - M2.M[i][j];
// 		}
// 	}

// 	return M_out;
// }

// // Transpose 3x3 matrix: M_out = M_in'
// Mat3x3 TransposeMat3x3(Mat3x3 M_in){
// 	Mat3x3 M_out;

// 	for (int i = 0; i < 3; i++){
// 		for (int j = 0; j < 3; j++){
// 			M_out.M[i][j] = M_in.M[j][i];
// 		}
// 	}

// 	return M_out;
// }

// Calculate Skew symmetric matrix from vector
// Mat3x3 skew(Vec3 V){
// 	Mat3x3 M;
// 	M.M[0][0] = 0;
// 	M.M[0][1] = -V(2);
// 	M.M[0][2] = V(1);

// 	M.M[1][0] = V(2);
// 	M.M[1][1] = 0;
// 	M.M[1][2] = -V(0);

// 	M.M[2][0] = -V(1);
// 	M.M[2][1] = V(0);
// 	M.M[2][2] = 0;

// 	return M;

// }

// Inverse operation for Skew symmetric matrices
Matrix<float, 3, 1> invSkew(Matrix<float, 3, 3> Mat){
	Matrix<float, 3, 1> w;

	w << -Mat(1,2),
		  Mat(0,2),
		 -Mat(0,1);

	return w;
}

// Vec3 invSkew(Mat3x3 Mat){
// 	Vec3 w;

// 	w(0) = -Mat.M[1][2];
// 	w(1) = Mat.M[0][2];
// 	w(2) = -Mat.M[0][1];

// 	return w;
// }

//Cross product between two vectors
// Vec3 cross(Vec3 V1, Vec3 V2){
	
// 	return MultiplyMat3x3Vec3(skew(V1), V2);

// }

//Calculate the p-norm of a 3x1 vector
// double p_normVec3(Vec3 V, int p){
// 	return pow(pow(V(0), p) + pow(V(1), p) + pow(V(2), p), 1.0 / p);
// }

Matrix<float, 3, 1> normalizeVec3(Matrix<float, 3, 1> V){
	double normV = V.norm();
	if (normV > 0){
		return V*(1.0 / normV);
	}
	else{
		return V;
	}
}

//Inner product between two matrices
// double innerProd(Vec3 V1, Vec3 V2){
// 	return V1(0)*V2(0) + V1(1)*V2(1) + V1(2)*V2(2);
// }

/* Print 3x3 matrices for debugging*/
// void PrintMat3x3(Mat3x3 Mat){
// 	for (int i = 0; i < 3; i++){
// 		for (int j = 0; j < 3; j++){
// 			printf("%f\t", Mat.M[i][j]);
// 		}
// 		printf("\n");
// 	}
// 	printf("\n");
// }

/* Print 4x4 matrices for debugging*/
void PrintMat4x4(Matrix<float, 4, 4> Mat){
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			printf("%f\t", Mat(i,j));
		}
		printf("\n");
	}
	printf("\n");
}

/* Print 3x1 vectors for debugging*/
void PrintVec3(Matrix<float, 3, 1> V, char const *Text){
	printf("%s = \t", Text);
	for (int i = 0; i < 3; i++){
		printf("%f \t", V(i));
	}
	printf("\n");
}

/* Print 4x1 vectors for debugging*/
void PrintVec4(Matrix<float, 4, 1> V, char const *Text){
	printf("%s = \t", Text);
	for (int i = 0; i < 4; i++){
		printf("%f \t", V(i));
	}
	printf("\n");
}

// Multiply 3x3 matrix by a 3x1 vectos: V_out = M*V
// Vec3 MultiplyMat3x3Vec3(Mat3x3 Mat, Vec3 V){
// 	Vec3 V_out;

// 	for (int i = 0; i < 3; i++){
// 		V_out.v[i] = 0;
// 		for (int k = 0; k < 3; k++){
// 			V_out.v[i] += Mat.M[i][k] * V.v[k];
// 		}
// 	}

// 	return V_out;
// }

// Scale 3x1 vector: V_out = c.V_in, where c is a constant
// Vec3 ScaleVec3(Vec3 V_in, float c){
// 	Vec3 V_out;

// 	for (int i = 0; i < 3; i++){
// 		V_out.v[i] = c*V_in.v[i];
// 	}

// 	return V_out;
// }

// Add 3x1 vectors: V_out = V1 + V2
// Vec3 Add3x1Vec(Vec3 V1, Vec3 V2){
// 	Vec3 V_out;
// 	V_out(0) = V1(0) + V2(0);
// 	V_out(1) = V1(1) + V2(1);
// 	V_out(2) = V1(2) + V2(2);

// 	return V_out;
// }

// Subtract 3x1 vectors: V_out = V1 - V2
// Vec3 Subtract3x1Vec(Vec3 V1, Vec3 V2){
// 	Vec3 V_out;
// 	V_out(0) = V1(0) - V2(0);
// 	V_out(1) = V1(1) - V2(1);
// 	V_out(2) = V1(2) - V2(2);

// 	return V_out;
// }

// Multiply 4x4 matrix by a 4x1 vector: V_out = M*V
// Vec4 MultiplyMat4x4Vec4(Mat4x4 Mat, Vec4 V){
// 	Vec4 V_out;

// 	for (int i = 0; i < 4; i++){
// 		V_out.v[i] = 0;
// 		for (int k = 0; k < 4; k++){
// 			V_out.v[i] += Mat.M[i][k] * V.v[k];
// 		}
// 	}

// 	return V_out;
// }

// //Concatenate three vectors into a 3x3 matrix
// Mat3x3 Concatenate3Vec3_2_Mat3x3(Vec3 V1, Vec3 V2, Vec3 V3){
// 	Mat3x3 M;
// 	M.M[0][0] = V1(0);
// 	M.M[1][0] = V1(1);
// 	M.M[2][0] = V1(2);

// 	M.M[0][1] = V2(0);
// 	M.M[1][1] = V2(1);
// 	M.M[2][1] = V2(2);

// 	M.M[0][2] = V3(0);
// 	M.M[1][2] = V3(1);
// 	M.M[2][2] = V3(2);

// 	return M;
// }


//Verify if any of the terms in a Vec3 is NaN
int isNanVec3(Matrix<float, 3, 1> V){
	if(isnan(V(0)) || isnan(V(1)) || isnan(V(2))){
		printf("Not a number found!\n");
		return 1;
	}
	else{
		return 0;
	}

}