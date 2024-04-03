//Copied mostly from here: https://github.com/pms67/Attitude-Estimation/blob/master/kalman_imu.py
//And implemented in C

#include "kalmanfilter.h"

float** C;
float** Q;
float** R;
float** S;
float** K;

float** E;
float** F;

float** gyro_input;
float** y_tilde;
float** measurement;
float** intermediary_identity;

float** A;
float** B;

float** dot1;
float** dot2;
float** dot3;
float** dot4;
float** dot5;
float** dot6;
float** dot7;
float** dot8;
float** dot9;
float** dot10;
float** dot11;
float** dot12;

float** transpose1;
float** transpose2;
float** transpose3;

float** inverse1;

void setupKalmanFilter(){
	initialize(&C, 2, 4);
	initialize(&Q, 4, 4);
	initialize(&R, 2, 2);
	initialize(&S, 2, 2);
	initialize(&K, 4, 2);

	initialize(&gyro_input, 2, 1);
	initialize(&y_tilde, 2, 1);
	initialize(&measurement, 2, 1);
	initialize(&intermediary_identity, 4, 4);

	initialize(&E, 3, 3);
	initialize(&F, 3, 3);

	identity(E, 3);
	identity(F, 3);

	initialize(&A, 4, 4);
	initialize(&B, 4, 2);

	C[0][0] = 1;
	C[1][2] = 1;

	identity(Q, 4);
	identity(R, 2);

	initialize(&dot1, 4, 4);
	initialize(&dot2, 4, 1);
	initialize(&dot3, 4, 4);
	initialize(&dot4, 4, 4);
	initialize(&dot5, 2, 1);
	initialize(&dot6, 4, 2);
	initialize(&dot7, 2, 2);
	initialize(&dot8, 2, 2);
	initialize(&dot8, 4, 2);
	initialize(&dot9, 4, 1);
	initialize(&dot10, 4, 4);
	initialize(&dot11, 4, 4);

	initialize(&transpose1, 4, 4);
	initialize(&transpose2, 4, 2);
	initialize(&transpose3, 4, 2);

	initialize(&inverse1, 2, 2);
}

void estimate (float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, double* roll, double* pitch, double dt, float** P, float** state_estimate, int count){
	double accelRoll = predictRollAccel(accelX, accelY, accelZ);
	double rollRate = calculateRollRate(*pitch, *roll, gyroX, gyroY, gyroZ);

	double accelPitch = predictPitchAccel(accelX, accelY, accelZ);
	double pitchRate = calculatePitchRate(*pitch, *roll, gyroX, gyroY, gyroZ);

	A[0][0] = 1;
	A[0][1] = -dt;
	A[1][1] = 1;
	A[2][2] = 1;
	A[2][3] = -dt;
	A[3][3] = 1;

	B[0][0] = dt;
	B[2][1] = dt;

	gyro_input[0][0] = rollRate;
	gyro_input[1][0] = pitchRate;
	
	dotProduct(dot1, A, 4, 4, state_estimate, 4, 1);
	dotProduct(dot2, B, 4, 2, gyro_input, 2, 1);

	addMatrices(dot1, dot2, 4, 1, true);

	copy(state_estimate, dot1, 4, 1);

	zero(dot1, 4, 4);
	zero(dot2, 4, 2);

	transpose(transpose1, A, 4, 4);
	dotProduct(dot3, P, 4, 4, transpose1, 4, 4);
	dotProduct(dot4, A, 4, 4, dot3, 4, 4);
	addMatrices(dot4, Q, 4, 4, true);
	copy(P, dot4, 4, 4);

	zero(dot3, 4, 4);
	zero(dot4, 4, 4);
	zero(transpose1, 4, 4);

	measurement[0][0] = accelRoll;
	measurement[1][0] = accelPitch;

	//Corrupted from state_estimate
	dotProduct(dot5, C, 2, 4, state_estimate, 4, 1);
	addMatrices(y_tilde, dot5, 2, 1, false);
	addMatrices(y_tilde, measurement, 2, 1, true);

	zero(dot5, 2, 1);

	//printf("---%d---\n", count);
	//printMatrix(y_tilde, 2, 1);

	//Not corrupted
	transpose(transpose2, C, 2, 4);

	//Corrupted from P
	//dot6 is a 4x2
	dotProduct(dot6, P, 4, 4, transpose2, 4, 2);

	zero(transpose2, 4, 2);

	//Corrupted from dot6
	dotProduct(dot7, C, 2, 4, dot6, 4, 2);

	zero(dot6, 4, 2);

	//dot7 is a 2x2
	S[0][0] = R[0][0] + dot7[0][0];
	S[0][1] = R[0][1] + dot7[0][1];
	S[1][0] = R[1][0] + dot7[1][0];
	S[1][1] = R[1][1] + dot7[1][1];

	zero(dot7, 2, 2);

	inverse2X2(S, inverse1);
	transpose(transpose3, C, 2, 4);
	//dot8 is a 4x2
	dotProduct(dot8, transpose3, 4, 2, inverse1, 2, 2);

	zero(inverse1, 2, 2);
	zero(transpose3, 4, 2);

	//K is a 4x2
	dotProduct(K, P, 4, 4, dot8, 4, 2);
	//dot9 is a 4x1

	//State estimate messes up measurement
	dotProduct(dot9, K, 4, 2, y_tilde, 2, 1);
	
	addMatrices(state_estimate, dot9, 4, 1, true);

	zero(dot8, 4, 2);
	zero(dot9, 4, 1);

	//dot10 is a 4x4
	dotProduct(dot10, K, 4, 2, C, 2, 4);
	
	identity(intermediary_identity, 4);
	addMatrices(intermediary_identity, dot10, 4, 4, false);
	//dot11 is a 4x4
	dotProduct(dot11, intermediary_identity, 4, 4, P, 4, 4);
	zero(intermediary_identity, 4, 4);

	copy(P, dot11, 4, 4);

	zero(dot10, 4, 4);
	zero(dot11, 4, 4);
	zero(y_tilde, 2, 1);
	zero(K, 4, 2);

	*roll = state_estimate[0][0];
	*pitch = state_estimate[2][0];
}