#ifndef MOTOR_H_
#define MOTOR_H_
#include "PID.h"
#include "PID_horz.h"

class Motor{
	public:
		static void motor_speed_navigation(float dist, float curr_heading, float dest_heading, float* l_motor_val, float* r_motor_val, long elapsedTime);
		static bool motor_speed_visual_servo(float dist, float curr_angle, float dest_angle, float* l_motor_val, float* r_motor_val, long elapsedTime);
};

#endif /* MOTOR_H_ */