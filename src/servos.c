#include "servos.h"

//static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
const struct pwm_dt_spec roll_servo = PWM_DT_SPEC_GET(DT_NODELABEL(roll_servo));
const struct pwm_dt_spec pitch_servo = PWM_DT_SPEC_GET(DT_NODELABEL(pitch_servo));

static const uint32_t min_pulse_continuous = 1000;
static const uint32_t max_pulse_continuous = 2000;
int middle_pad = 75;


float move_roll_servo(float PID_output){
	int pwm_range = max_pulse_continuous - min_pulse_continuous;
	int half_range = pwm_range / 2 - middle_pad;

	int middle = (max_pulse_continuous + min_pulse_continuous) / 2;
	int lower_middle = middle - middle_pad;
	int higher_middle = middle + middle_pad;

	//uint32_t pulse_width = min_pulse + pwm_range * angle / 120;
	if(PID_output > 0){
		uint32_t pulse_width = higher_middle + half_range * PID_output / 5;
	} else{
		uint32_t pulse_width = lower_middle + half_range * PID_output / 5;
	}

	if(pulse_width > max_pulse_continuous){
		pulse_width = max_pulse_continuous;
	} else if (pulse_width < min_pulse_continuous){
		pulse_width = min_pulse_continuous;
	}

	int ret = pwm_set_pulse_dt(&roll_servo, PWM_USEC(pulse_width));
	if (ret < 0) {
		printk("Error %d: failed to set pulse width\n", ret);
	}

	return pulse_width;
}

float move_pitch_servo(float PID_output){
	int pwm_range = max_pulse_continuous - min_pulse_continuous;
	int half_range = pwm_range / 2 - middle_pad;

	int middle = (max_pulse_continuous + min_pulse_continuous) / 2;
	int lower_middle = middle - middle_pad;
	int higher_middle = middle + middle_pad;

	//uint32_t pulse_width = min_pulse + pwm_range * angle / 120;
	if(PID_output > 0){
		uint32_t pulse_width = higher_middle + half_range * PID_output / 5;
	} else{
		uint32_t pulse_width = lower_middle + half_range * PID_output / 5;
	}

	if(pulse_width > max_pulse_continuous){
		pulse_width = max_pulse_continuous;
	} else if (pulse_width < min_pulse_continuous){
		pulse_width = min_pulse_continuous;
	}

	int ret = pwm_set_pulse_dt(&pitch_servo, PWM_USEC(pulse_width));
	if (ret < 0) {
		printk("Error %d: failed to set pulse width\n", ret);
	}

	return pulse_width;
}

void setup_roll_servo(){
	if (!device_is_ready(roll_servo.dev)) {
		printk("Error: PWM device %s is not ready\n", roll_servo.dev->name);
	}
	pwm_set_pulse_dt(&roll_servo, PWM_USEC(1500));
	printk("Roll servo configured.\n");
}

void setup_pitch_servo(){
	if (!device_is_ready(pitch_servo.dev)) {
		printk("Error: PWM device %s is not ready\n", pitch_servo.dev->name);
	}
	pwm_set_pulse_dt(&pitch_servo, PWM_USEC(1500));
	printk("Pitch servo configured.\n");
}
