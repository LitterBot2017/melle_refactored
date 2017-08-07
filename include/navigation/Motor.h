#ifndef MOTOR_H_
#define MOTOR_H_
#include "PID.h"
#include "PID_horz.h"

class Motor{
	public:
		static void motor_speed_navigation(float dist, float curr_heading, float dest_heading, float* l_motor_val, float* r_motor_val, long elapsedTime);
		static bool motor_speed_visual_servo(float dist, float curr_angle, float dest_angle, float* l_motor_val, float* r_motor_val, long elapsedTime);
		static void motor_speed_joystick(float angular_input, float linear_input, int max_speed_teleop, float* left_motor, float* right_motor);
		static void move_forward_blind(float* left_motor, float* right_motor);
		static void motor_stop(float* left_motor, float* right_motor);
		static bool motor_turn(float x, float y, float x_center, float y_center,float* left_motor, float* right_motor);
};

#endif /* MOTOR_H_ */