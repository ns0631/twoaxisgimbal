#include "helpermath.h"

double predictPitchAccel(float ax, float ay, float az);

double predictRollAccel(float ax, float ay, float az);

double calculatePitchRate(double pitch, double roll, float gyroX, float gyroY, float gyroZ);

double calculateRollRate(double pitch, double roll, float gyroX, float gyroY, float gyroZ);

double calculatePitch(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, double pitch, double roll, double dt);

double calculateRoll(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, double pitch, double roll, double dt);