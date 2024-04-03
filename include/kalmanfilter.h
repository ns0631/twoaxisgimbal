#include "matrix.h"
#include "complementaryfilter.h"

struct attitude{
	float pitch;
	float roll;
};

void estimate (float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, double* roll, double* pitch, double dt, float** P, float** state_estimate, int count);

void setupKalmanFilter();