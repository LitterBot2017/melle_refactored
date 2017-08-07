#include "Motor.h"
#include "math.h"
#include "PID.h"
#include "PID_horz.h"
#include <algorithm>

using namespace std;

//////Check these in navigation
#define MAX_TURNING_SPEED_NAVIGATION 8
#define MAX_FORWARD_SPEED_NAVIGATION 20
PID turn_pid_navigation = PID(0, 10, 0.01, 0, 0, MAX_TURNING_SPEED_NAVIGATION, -1 * MAX_TURNING_SPEED_NAVIGATION);
PID_horz forward_pid_navigation = PID_horz(0, 50, 10, 0, 0, MAX_FORWARD_SPEED_NAVIGATION, -1 * MAX_FORWARD_SPEED_NAVIGATION);
//////
#define MAX_TURNING_SPEED_SERVO 4 
#define MAX_FORWARD_SPEED_SERVO 5
PID turn_pid_servo = PID(0, 0.5, 0.0002, 0, 0, MAX_TURNING_SPEED_SERVO, -1 * MAX_TURNING_SPEED_SERVO);
PID_horz forward_pid_servo = PID_horz(0, 0.2, 0.0002, 0, 0, MAX_FORWARD_SPEED_SERVO, -1 * MAX_FORWARD_SPEED_SERVO);

void Motor::motor_speed_navigation(float dist, float curr_heading, float dest_heading,float* left_motor, float* right_motor, long elapsedTime)
{
	float turn_speed = turn_pid_navigation.getNewValue(curr_heading, dest_heading, elapsedTime);
	float forward_speed = forward_pid_navigation.getNewValue(dist, elapsedTime);
	turn_speed = turn_speed/2;
	forward_speed = forward_speed/2;
	*left_motor = 64 - forward_speed - turn_speed;
	*right_motor = 64 - forward_speed + turn_speed;

	// Clip motor speeds to be between 39 and 89
	*left_motor = std::max(float(39.0), std::min(float(*left_motor), float(89.0)));
	*right_motor = std::max(float(39.0), std::min(float(*right_motor), float(89.0)));

	if (dist < 3) {
		*right_motor = 64;
		*left_motor = 64;
	}

}

bool Motor::motor_speed_visual_servo(float dist, float curr_angle, float dest_angle, float* left_motor, float* right_motor, long elapsedTime)
{
	float turn_speed = turn_pid_servo.getNewValue(curr_angle, dest_angle, elapsedTime/1000);
	float forward_speed = forward_pid_servo.getNewValue(dist, elapsedTime/1000);
	turn_speed = turn_speed;
	forward_speed = forward_speed;
	*left_motor = 64 - forward_speed - turn_speed;
	*right_motor = 64 - forward_speed + turn_speed;

	// Clip motor speeds to be between 39 and 89
	*left_motor = std::max(float(39.0), std::min(float(*left_motor), float(89.0)));
	*right_motor = std::max(float(39.0), std::min(float(*right_motor), float(89.0)));

	if (abs((int)dist) < 10 ) {
		*right_motor = 64;
		*left_motor = 64;
		return true;
	}

	return false;

}

void Motor::motor_speed_joystick(float angular_input, float linear_input, int max_speed_teleop,float* left_motor, float* right_motor )
{
	float turn_speed = angular_input*8;
	float forward_speed = linear_input*max_speed_teleop;
	*left_motor = (64 + forward_speed - turn_speed);
	*right_motor = (64 + forward_speed + turn_speed);
}

void Motor::move_forward_blind(float* left_motor, float* right_motor)
{
	*left_motor = 71;
	*right_motor = 70;
}

void Motor::motor_stop(float* left_motor, float* right_motor)
{
	*left_motor = 64;
	*right_motor = 64;
}

bool Motor::motor_turn(float x, float y, float x_center, float y_center,float* left_motor, float* right_motor)
{
	if(x-x_center>50)
	{
		*left_motor =  64 + 6;
  		*right_motor = 64 - 6;
	}
	else if(x-x_center<50)
	{
		*left_motor =  64 - 6;
  		*right_motor = 64 + 6;
	}
	if(x<x_center+50 && x>x_center-50)
	{
		if(y-y_center>50)
		{
			*left_motor =  64 - 9;
  			*right_motor = 64 - 9;
		}
		else if(y-y_center<50)
		{
			*left_motor =  64 + 9;
  			*right_motor = 64 + 9;
		}
	}
	if(x<x_center+50 && x>x_center-50 && y<y_center+50 && y>y_center-50)
	{
		return true;
	}	
	return false;
}