#include "Visual_Servo.h"
#include "math.h"

using namespace std;

float Visual_Servo::calculate_distance(float x, float y, float x_center, float y_center)
{
	float xdiff = x-x_center;
	float ydiff = y-y_center;
	float dist = sqrt((xdiff*xdiff)+(ydiff*ydiff));
	if(ydiff>0)
		return -dist;
	return dist;
}

float Visual_Servo::calculate_angle(float x, float y, float x_center, float y_center, float init_angle)
{
	float xdiff = x-x_center;
	float ydiff = y-y_center;
	float angle = atan2(ydiff,xdiff);
	return init_angle+angle;
}