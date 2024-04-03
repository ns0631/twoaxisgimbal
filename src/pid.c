//Mostly copied from here: https://wired.chillibasket.com/2015/03/pid-controller/

#include "pid.h"

// Declare variables
static float Kp = 1e-1;          // (P)roportional Tuning Parameter
static float Ki = 5e-3;          // (I)ntegral Tuning Parameter
static float Kd = 0;          // (D)erivative Tuning Parameter
float iTerm = 0;       // Used to accumulate error (integral)
float lastTime = 0;    // Records the time the function was last called
static float maxPID = 5;    // The maximum value that can be output
float target = 0;

float oldPitch = 0;    // The last sensor value
float oldRoll = 0;    // The last sensor value

float pidPitch(float current, double dT) {
	// Calculate error between target and current values
	float error = target - current;

	// Calculate the integral term
	iTerm += error * dT;

	// Calculate the derivative term (using the simplification)
	float dTerm = (current - oldPitch) / dT;

	// Set old variable to equal new ones
	oldPitch = current;

	// Multiply each term by its constant, and add it all up
	float result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);

	// Limit PID value to maximum values
	if (result > maxPID) result = maxPID;
	else if (result < -maxPID) result = -maxPID;
	//printk("P term: %f; I term: %f; D term: %f; error: %f; result: %f\n", error * Kp, (iTerm * Ki), (dTerm * Kd), error, result);

	return result;
}


float pidRoll(float current, double dT) {
	// Calculate error between target and current values
	float error = target - current;

	// Calculate the integral term
	iTerm += error * dT;

	// Calculate the derivative term (using the simplification)
	float dTerm = (current - oldRoll) / dT;

	// Set old variable to equal new ones
	oldPitch = current;

	// Multiply each term by its constant, and add it all up
	float result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);

	// Limit PID value to maximum values
	if (result > maxPID) result = maxPID;
	else if (result < -maxPID) result = -maxPID;
	//printk("P term: %f; I term: %f; D term: %f; error: %f; result: %f\n", error * Kp, (iTerm * Ki), (dTerm * Kd), error, result);

	return result;
}
