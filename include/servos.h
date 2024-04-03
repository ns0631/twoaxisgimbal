#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>


void setup_roll_servo();
void setup_pitch_servo();

float move_roll_servo(float PID_output);
float move_pitch_servo(float PID_output);
