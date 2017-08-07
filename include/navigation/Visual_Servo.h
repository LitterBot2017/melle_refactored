#ifndef VISUAL_SERVO_H_
#define VISUAL_SERVO_H_

class Visual_Servo{
	public:
		static float calculate_distance(float x, float y, float x_center, float y_center);
		static float calculate_angle(float x, float y, float x_center, float y_center);
};

#endif /* VISUAL_SERVO_H_ */