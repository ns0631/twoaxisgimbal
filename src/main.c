/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
Arduino code, adapted for zephyr: https://mschoeffler.com/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/
Following tutorial from here: https://academy.nordicsemi.com/topic/i2c-driver/
For complementary filter: https://www.hackster.io/hibit/complementary-filter-and-relative-orientation-with-mpu9250-d4f79d#toc-gyroscope-angles-3
Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <stdlib.h>

#include "kalmanfilter.h"
#include "complementaryfilter.h"
#include "servos.h"
#include "pid.h"

/* The devicetree node identifier for the MPU6050 imu */
#define I2C0_NODE DT_NODELABEL(imu)

//define MPU6050_TEMP_HIGH_REG 0x41
//define MPU6050_TEMP_LOW_REG 0x42
#define MPU6050_PWR_REG 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define LOW_PASS_FILTER_CONFIG 0x1A

#define tempLowRegister 0x42
#define tempHighRegister 0x41

#define gyroXHighRegister 0x43
#define gyroXLowRegister 0x44
#define gyroYHighRegister 0x45
#define gyroYLowRegister 0x46
#define gyroZHighRegister 0x47
#define gyroZLowRegister 0x48

#define accelXHighRegister 0x3B

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

static float offsets[] = {1.33426739, -0.54700182, -0.77473574,  0.15876233,  1.22315467, 1.37050595};

int main(void)
{
	int ret;

	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return 1;
	}

	//Configure
	uint8_t pwr[2] = {MPU6050_PWR_REG,0x0};
	ret = i2c_write_dt(&dev_i2c, pwr, sizeof(pwr));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,pwr[0]);
		return 0;
	}

	int8_t temp[2];
	int8_t gyro_reading[6];
	int8_t accel_reading[6];
	//uint8_t sensor_regs[2] ={MPU6050_TEMP_LOW_REG,MPU6050_TEMP_HIGH_REG};

	int success = 1;

	float trueTemp, trueTempF;
	float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

	//Write to address 1B to configure gyroscope
	uint8_t config[2] = {MPU6050_GYRO_CONFIG,0x6};
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		return 0;
	}

	//Change config over to accel
	config[0] = MPU6050_ACCEL_CONFIG;
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		return 0;
	}

	//Enable digital low pass filter (random setting idk, page 13 of register map)
	config[0] = LOW_PASS_FILTER_CONFIG;
	config[1] = 0x6;
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		return 0;
	}

	int64_t current_time;
	int64_t prev_time = k_uptime_get();
	int64_t last_time = k_uptime_get();

	double pitch = 0;
	double roll = 0;

	float** P;
	initialize(&P, 4, 4);
	identity(P, 4);

	float** state_estimate;
	initialize(&state_estimate, 4, 1);

	float pidRollMotorOutput, pidPitchMotorOutput;

	double dt;
	int count = 0;

	setup_roll_servo();
	setup_pitch_servo();
	setupKalmanFilter();

	float pw1, pw2;

	while (1) {
		success = 1;
		ret = i2c_burst_read_dt(&dev_i2c, tempHighRegister, temp, sizeof(temp));
		if(ret != 0){
			printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr, tempHighRegister);
			success = 0;
		}
		trueTemp = (float) (temp[0] * 256 + temp[1]) / 340 + 36.53;
		trueTempF = 1.8 * trueTemp + 32;

		ret = i2c_burst_read_dt(&dev_i2c, gyroXHighRegister, gyro_reading, sizeof(gyro_reading));
		if(ret != 0){
			printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr, gyroXHighRegister);
			success = 0;
		}
		ret = i2c_burst_read_dt(&dev_i2c, accelXHighRegister, accel_reading, sizeof(accel_reading));
		if(ret != 0){
			printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr, accelXHighRegister);
			success = 0;
		}

		gyroX = (float) gyro_reading[0] * 256 + gyro_reading[1];
		gyroY = (float) gyro_reading[2] * 256 + gyro_reading[3];
		gyroZ = (float) gyro_reading[4] * 256 + gyro_reading[5];

		gyroX /= 131;
		gyroY /= 131;
		gyroZ /= 131;

		accelX = (float) accel_reading[0] * 256 + accel_reading[1];
		accelY = (float) accel_reading[2] * 256 + accel_reading[3];
		accelZ = (float) accel_reading[4] * 256 + accel_reading[5];

		accelX /= 16384;
		accelY /= 16384;
		accelZ /= 16384;

		accelX *= g;
		accelY *= g;
		accelZ *= g;

		gyroX += offsets[0];
		gyroY += offsets[1];
		gyroZ += offsets[2];
		accelX += offsets[3];
		accelY += offsets[4];
		accelZ += offsets[5];

		gyroX *= PI / 180.;
		gyroY *= PI / 180.;
		gyroZ *= PI / 180.;

		//Round all to nearest 2 decimal places
		accelX = roundf(accelX * 100) / 100;
		accelY = roundf(accelY * 100) / 100;
		accelZ = roundf(accelZ * 100) / 100;

		gyroX = roundf(gyroX * 100) / 100;
		gyroY = roundf(gyroY * 100) / 100;
		gyroZ = roundf(gyroZ * 100) / 100;

		//printk("started!\n");
		if(success){
			//printk("Angular velocity:\nX: %f\nY: %f\nZ: %f\nAcceleration:\nX: %f\nY: %f\nZ: %f\nFahrenheit: %f; Celsius: %f; %d %d\n", gyroX, gyroY, gyroZ, accelX, accelY, accelZ, trueTempF, trueTemp, temp[0], temp[1]);
			current_time = k_uptime_get();
			dt = (double) (current_time - prev_time) / 1000;
			prev_time = current_time;

			estimate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, &roll, &pitch, dt, P, state_estimate, count);

			pidRollMotorOutput = pidRoll(roll * 180 / PI, dt);
			pidPitchMotorOutput = pidPitch(pitch * 180 / PI, dt);

			pw1 = move_roll_servo(pidRollMotorOutput);
			pw2 = move_pitch_servo(-pidPitchMotorOutput);

			//printk("%d\n", success);
			if(current_time - last_time > 500){
				printk("Roll: %.1lf | Pitch: %.1lf | PID roll output: %.2f | Roll pulse width: %.1f | PID pitch output: %.2f | Pitch pulse width: %.1f\n", roll * 180 / PI, pitch * 180 / PI, pidRollMotorOutput, pw1, pidPitchMotorOutput, pw2);
				last_time = current_time;
			}
		}
		//k_msleep(500);
	}
	return 0;
}
