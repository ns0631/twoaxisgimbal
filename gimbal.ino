#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <Servo.h>

const int AHRS_ADDR = 0x50; // I2C address of the AHRS

int16_t roll, pitch, yaw;
int error = 0;
int resetPin = 12;

int iter = 0;

Servo rollServo;
Servo pitchServo;

static float Kp = 4.5e-2;          // (P)roportional Tuning Parameter
static float Ki = 0;          // (I)ntegral Tuning Parameter
static float Kd = 5e-5;          // (D)erivative Tuning Parameter

float pitchIntegral = 0;       // Used to accumulate error (integral)
float rollIntegral = 0;

float oldPitchError = 0;
float oldRollError = 0;
float targetPitch = 0;
float targetRoll = 90;

uint32_t oldTime = 0;
uint32_t newTime;
float dt;

float pidPitch(float current, float dt) {
	float error = targetPitch - current;
	pitchIntegral += error * dt;
	float derivative = (error - oldPitchError) / dt;
	oldPitchError = error;
	float result = (error * Kp) + (pitchIntegral * Ki) + (derivative * Kd);
	return result;
}

float pidRoll(float current, float dt) {
	float error = targetRoll - current;
	rollIntegral += error * dt;
	float derivative = (error - oldRollError) / dt;
	oldRollError = error;
	float result = (error * Kp) + (rollIntegral * Ki) + (derivative * Kd);
	return result;
}

int pidToPulse(float pid){
  int pulse;

  if(pid > 0){
    pulse = (int) (1530. + 470. * pid / 5.);
  } else{
    pulse = (int) (1470. + 470. * pid / 5.);
  }

  if(pulse > 2000){
    pulse = 2000;
  } else if (pulse < 1000){
    pulse = 1000;
  }

  return pulse;
}

void setup() {
  digitalWrite(resetPin, HIGH);
  delay(200);
  pinMode(resetPin, OUTPUT);  
  
  Serial.begin(9600);
  Serial.println("Reset.");

  Wire.begin();
  Wire.beginTransmission(AHRS_ADDR);
  Wire.endTransmission(true);

  rollServo.attach(9);
  pitchServo.attach(5);

  rollServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);

  oldTime = millis();
}

void loop() {
  Wire.beginTransmission(AHRS_ADDR);
  Wire.write(0x3d); 
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(AHRS_ADDR, 3*2, true); // request a total of 6 registers
  
  roll = Wire.read() + Wire.read() * 256;
  pitch = Wire.read() + Wire.read() * 256;
  yaw = Wire.read() + Wire.read() * 256;

  newTime = millis();
  dt = (float) ( 0.001 * (newTime - oldTime) ) ;

  float yaw_measurement = ( (float) yaw ) / 32768. * 180.;
  float pitch_measurement = ( (float) pitch ) / 32768. * 180.;
  float roll_measurement = ( (float) roll ) / 32768. * 180.;
  
  float pitchPIDOutput = pidPitch(pitch_measurement, dt);
  float rollPIDOutput = pidRoll(roll_measurement, dt);

  int pitchPulse = pidToPulse(pitchPIDOutput);
  int rollPulse = pidToPulse(-rollPIDOutput);

  if(iter % 250 == 0){
    Serial.print("roll = "); Serial.print( roll_measurement );
    Serial.print(" | pitch = "); Serial.print( pitch_measurement );
    Serial.print(" | yaw = "); Serial.println( yaw_measurement );

    Serial.print("roll PID output = "); Serial.print( rollPIDOutput );
    Serial.print(" | pitch PID output = "); Serial.print( pitchPIDOutput );
    Serial.println();

    Serial.print("roll pulse = "); Serial.print( rollPulse );
    Serial.print(" | pitch pulse = "); Serial.print( pitchPulse );
    Serial.println();
    //Serial.println( yaw_measurement );
  }

  //pitchServo.writeMicroseconds( (int) ( 1500 + pitch_measurement / 90. * 500) );

  pitchServo.writeMicroseconds(pitchPulse);
  rollServo.writeMicroseconds(rollPulse);

  //Serial.println((int) ( 1500 + pitch_measurement / 90. * 500));

  if(pitch_measurement > -1.43 && pitch_measurement < -1.40 && roll_measurement > -1.43 && roll_measurement < -1.40){
    error++;
  }

  if(error %= 5){
    digitalWrite(resetPin, LOW);
  }

  oldTime = newTime;
  iter++;
}