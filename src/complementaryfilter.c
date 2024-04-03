//Taken from this video: https://www.youtube.com/watch?v=BUW2OdAtzBw
//And this tutorial: https://www.hackster.io/hibit/complementary-filter-and-relative-orientation-with-mpu9250-d4f79d
#include "complementaryfilter.h"

double predictPitchAccel(float ax, float ay, float az){
	//double pitch = atan2(ax, g);
	double magnitude = sqrt( pow( ax, 2 ) + pow( az, 2 ) );
	double pitch = atan2(ay, magnitude);
	return pitch;
}

double predictRollAccel(float ax, float ay, float az){
	//double roll = atan2(ay, az);
	double magnitude = sqrt( pow( ay, 2 ) + pow( az, 2 ) );
	double roll = atan2(-ax, magnitude);
	return roll;
}

double calculateRollRate(double pitch, double roll, float gyroX, float gyroY, float gyroZ){
	double a = 1;
	double b = calculateSin(roll) * calculateTangent(pitch);
	double c = calculateCos(roll) * calculateTangent(pitch);

	double rollRate = a * gyroX + b * gyroY + c * gyroZ;
	return rollRate;
}

double calculatePitchRate(double pitch, double roll, float gyroX, float gyroY, float gyroZ){
	//double a = 0;
	double b = calculateCos(roll);
	double c = -calculateSin(roll);

	//double pitchRate = a * gyroX + b * gyroY + c * gyroZ;
	double pitchRate = b * gyroY + c * gyroZ;
	return pitchRate;
}

double calculateRoll(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, double pitch, double roll, double dt){
	double accelRoll = predictRollAccel(accelX, accelY, accelZ);
	double rollRate = calculateRollRate(pitch, roll, gyroX, gyroY, gyroZ);
	roll = alpha * accelRoll + (1 - alpha) * (roll + dt * rollRate);
	
	return roll;
}

double calculatePitch(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, double pitch, double roll, double dt){
	double accelPitch = predictPitchAccel(accelX, accelY, accelZ);
	double pitchRate = calculatePitchRate(pitch, roll, gyroX, gyroY, gyroZ);
	pitch = alpha * accelPitch + (1 - alpha) * (pitch + dt * pitchRate);
	
	return pitch;
}