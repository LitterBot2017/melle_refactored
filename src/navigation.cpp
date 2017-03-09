#include "ros/ros.h"

#include "arduino_pc/Arduino.h"

#include "navigation/Navigation.h"
#include "navigation/Debug.h"
#include "navigation/Arm.h"

#include "object_tracker/Position.h"
#include "obstacle_avoidance/DesiredHeading.h"
#include "yolo2/ImageDetections.h"
#include "yolo2/Detection.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Joy.h>

#include "GPS.h"
#include "PID.h"
#include "PID_horz.h"

using namespace std;

// States
#define GET_GPS_LOCK 1
#define MOVE_TO_WAYPOINT 2
#define JOYSTICK 3
#define FORWARD_SERVO 4
#define MOVE_TO_DOWNWARD 5
#define DOWNWARD_SERVO 6
#define CLASSIFICATION 7
#define ARM_PICKUP 8

// Camera Indices
#define DOWNWARD_CAMERA 0
#define FORWARD_CAMERA 1

// Image sizes
#define DOWNWARD_WIDTH 1280
#define DOWNWARD_HEIGHT 470
#define FORWARD_WIDTH 1280
#define FORWARD_HEIGHT 720
#define SERVO_TOLERANCE 50

// Litter Classification Confidence Threshold
#define MIN_CONFIDENCE 0.80

int curr_state = GET_GPS_LOCK;

//GPS_Waypoints before 2/26/2017
//float lat_list [] = { 40.4421768188, 40.4420509338, 40.442111969};
//float long_list [] = { -79.9453964233, -79.9453735352,-79.9455795288 };
//GPS_Waypoints after 2/26/2017, more centered on second level
float lat_list [] = { 40.442222, 40.442072};
float long_list [] = { -79.945563, -79.945515};

int curr_ind = 0;

float curr_lat;
float curr_long;
float curr_heading;
float dest_lat = lat_list[curr_ind];
float dest_long = long_list[curr_ind];
long elapsedTime;
float dis_to_dest;
float head_to_dest;
float obs_direction;
float obs_magnitude;
float obs_magnitude_modifier = 30;
float speed;
int sats;

int max_speed_teleop = 20;

//Diagnostic data
float batt_level;
float bin_fullness;
string downview_state;

//Debug mode
bool enable_debug_mode = false;

//Subscribers
ros::Subscriber arduino_sub;
ros::Subscriber arm_state_sub;
ros::Subscriber joystick_sub;
ros::Subscriber obav_desired_heading_sub;
ros::Subscriber object_track_sub;
ros::Subscriber yolo_sub;

//Publishers
ros::Publisher navigation_pub;
ros::Publisher debug_pub;
ros::Publisher camera_select_pub;
ros::Publisher arm_pub;

//Publisher msgs
navigation::Navigation navigation_msg;
navigation::Debug debug_msg;
navigation::Arm arm_msg;

//Motor_controller calculator and PID
#define MAX_TURNING_SPEED 20
#define MAX_FORWARD_SPEED 50
PID turn_pid = PID(0, 10, 0.01, 0, 0, MAX_TURNING_SPEED, -1 * MAX_TURNING_SPEED);
PID_horz forward_pid = PID_horz(0, 10, 0.01, 0, 0, MAX_FORWARD_SPEED, -1 * MAX_FORWARD_SPEED);
float left_motor = 64;
float right_motor = 64;

//Debug message creator
void publish_debug_message() {

    if(enable_debug_mode) {

		debug_msg.curr_lat = curr_lat;
		debug_msg.curr_long = curr_long;
		debug_msg.elapsed_time = elapsedTime;
		debug_msg.sats = sats;
		debug_msg.speed_val = speed;
		debug_msg.heading = curr_heading;
		debug_msg.bin_fullness = bin_fullness;
		debug_msg.battery = batt_level;
		debug_msg.l_motor = left_motor;
		debug_msg.r_motor = right_motor;
		debug_msg.dest_lat = dest_lat;
		debug_msg.dest_long = dest_long;
		debug_msg.waypoint_id = curr_ind;
		debug_msg.head_to_dest = head_to_dest;
		debug_msg.dist_to_dest = dis_to_dest;
		debug_msg.head_error = curr_heading-head_to_dest;
		debug_msg.current_state = curr_state;
		debug_msg.downview_state = downview_state;
		debug_msg.obs_modifier = obs_magnitude_modifier;

		debug_pub.publish(debug_msg);
	}
}

void select_camera(int camera) {
  	std_msgs::Int8 msg;
  	msg.data = camera;
    camera_select_pub.publish(msg);
}

void calculate_motor_speed()
{
	
	if(curr_state != GET_GPS_LOCK && curr_state != MOVE_TO_WAYPOINT) {
		navigation_msg.waypoint_id = 40;
		return;
	}

	else if(curr_state == GET_GPS_LOCK) {
		navigation_msg.waypoint_id = 50;
		left_motor = 64;
		right_motor = 64;
		return;
	}

	else if(curr_state == MOVE_TO_WAYPOINT) {

		float turn_speed = turn_pid.getNewValue(curr_heading, head_to_dest, elapsedTime);
		float forward_speed = forward_pid.getNewValue(dis_to_dest, elapsedTime);
		//debug_msg.r_motor=turn_speed;
		//debug_msg.l_motor=forward_speed;
		turn_speed = turn_speed/2;
		forward_speed = forward_speed/2;
		left_motor = 64 - forward_speed - turn_speed;
		right_motor = 64 - forward_speed + turn_speed;

		// Clip motor speeds to be between 39 and 89
		left_motor = std::max(float(39.0), std::min(float(left_motor), float(89.0)));
		right_motor = std::max(float(39.0), std::min(float(right_motor), float(89.0)));

		if (dis_to_dest < 3) {
			right_motor = 64;
			left_motor = 64;
		}
	}
}

bool turn_to_center(float x_pos, float y_pos,
	float width, float height, float tolerance,
	float* motor_l, float* motor_r) {

	if(x_pos - (width/2) > (tolerance/2)) {
		left_motor =  64 + 4;
  		right_motor = 64 - 4;
	} else if(x_pos - (width/2) < (-1*tolerance/2)) {
		left_motor =  64 - 4;
  		right_motor = 64 + 4;
	} else if(y_pos - (height/2) > (tolerance/2)) {
		left_motor =  64 - 3;
		right_motor = 64 - 3;
	} else if(y_pos - (height/2) < (-1 * tolerance/2)) {
		left_motor =  64 + 3;
		right_motor = 64 + 3;
	} else {
		left_motor = 64;
		right_motor = 64;
		return true;
	}
	return false;
}

//Object Track Callback
void object_track_callback(const object_tracker::Position msg) {

	if (curr_state != FORWARD_SERVO && curr_state != DOWNWARD_SERVO && curr_state != ARM_PICKUP)
		return;

	if (curr_state == FORWARD_SERVO) {
		bool servo_completed = turn_to_center(msg.x, msg.y,
			FORWARD_WIDTH, FORWARD_HEIGHT, SERVO_TOLERANCE, &left_motor, &right_motor);
		if (servo_completed) {
			curr_state = MOVE_TO_DOWNWARD;
			select_camera(DOWNWARD_CAMERA);
			left_motor =  64 + 3;
			right_motor = 64 + 3;
		}
	}

	if (curr_state == DOWNWARD_SERVO) {
		bool servo_completed = turn_to_center(msg.x, msg.y,
			DOWNWARD_WIDTH, DOWNWARD_HEIGHT, SERVO_TOLERANCE, &left_motor, &right_motor);
		if (servo_completed) {
			curr_state = ARM_PICKUP;
		}
	}

	if (curr_state == ARM_PICKUP) {
		navigation::Arm arm_msg;
		arm_msg.x = msg.x;
		arm_msg.y = msg.y;
		arm_msg.is_centered = "centered";
		navigation_msg.relay_state = true;
		left_motor = 64;
		right_motor = 64;
		arm_pub.publish(arm_msg);
	}
}

// Arduino Callback
void arduino_callback(const arduino_pc::Arduino arduino_msg)
{
	curr_heading = arduino_msg.heading;
	curr_long = arduino_msg.curr_long;
	curr_lat = arduino_msg.curr_lat;
	speed = arduino_msg.speed_val;
	sats = arduino_msg.sats;

	navigation_msg.waypoint_id = 10;
	
	elapsedTime = elapsedTime - arduino_msg.elapsed_time;
	
	dis_to_dest = GPS::distanceBetween(curr_lat, curr_long, dest_lat, dest_long);
	head_to_dest = GPS::courseTo(curr_lat, curr_long, dest_lat, dest_long)-((obs_magnitude/obs_magnitude_modifier)*(obs_direction));
	
	if (sats != 1 && (curr_state == GET_GPS_LOCK || curr_state == MOVE_TO_WAYPOINT)) {
		curr_state = GET_GPS_LOCK;
		navigation_msg.waypoint_id = 20;
	}
	if (sats == 1 && (curr_state == GET_GPS_LOCK || curr_state == MOVE_TO_WAYPOINT)) {
		curr_state = MOVE_TO_WAYPOINT;
		select_camera(FORWARD_CAMERA);
		navigation_msg.waypoint_id = 30;		
	}
	calculate_motor_speed();
	if(dis_to_dest < 3) {
		curr_ind++;
		dest_lat = lat_list[curr_ind % 2];
		dest_long = long_list[curr_ind %2];
	}

	batt_level = arduino_msg.battery;
	bin_fullness = arduino_msg.bin_fullness;
	if(arduino_msg.pickup_state) {
		arm_msg.pickup_state = "on";
	} else {
		arm_msg.pickup_state = "off";
	}
}

//Joystick callback to switch from twist message to left and right motor commands
void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if(joy->buttons[0]) {
		curr_state = GET_GPS_LOCK;
	} else if(joy->buttons[1]){
		// curr_state = OBSTACLE_AVOIDANCE;
	} else if(joy->buttons[2]) {
		curr_state = JOYSTICK;
	} else if(joy->buttons[3]) {
		// curr_state=OBJECT_TRACK;
	} else if(joy->buttons[9]) {
		if(enable_debug_mode)
		{
			enable_debug_mode=false;
		}
		else
		{
			enable_debug_mode=true;
		}
	}
	else if(joy->buttons[15])
		obs_magnitude_modifier+=2.5;
	else if(joy->buttons[16])
		obs_magnitude_modifier-=2.5;

	if(curr_state != JOYSTICK)
	{
		return;
	}
    if(curr_state == JOYSTICK && joy->buttons[7])
    	max_speed_teleop ++;
    if(curr_state == JOYSTICK && joy->buttons[6])
    	max_speed_teleop --;

	float turn_speed = joy->axes[2]*8;
	float forward_speed = joy->axes[1]*max_speed_teleop;
	left_motor = (64 + forward_speed - turn_speed);
	right_motor = (64 + forward_speed + turn_speed);
}

//yolo_callback for detections
void yolo_callback(const yolo2::ImageDetections detection_msg) {
	if (detection_msg.num_detections > 0) {
		if (curr_state == MOVE_TO_WAYPOINT) {
			curr_state = FORWARD_SERVO;
		} else if (curr_state == MOVE_TO_DOWNWARD) {
			curr_state = DOWNWARD_SERVO;
		} else if (curr_state == CLASSIFICATION) {
			bool isLitter = false;
			int classID;
			float confidence;
			for (int i = 0; i < detection_msg.num_detections; i++) {
				classID = detection_msg.detections[i].class_id;
				confidence = detection_msg.detections[i].confidence;
				if ((classID == 0 || classID == 1) && confidence > MIN_CONFIDENCE)
					curr_state = ARM_PICKUP;
			}
			if (curr_state != ARM_PICKUP) {
				curr_state = MOVE_TO_WAYPOINT;
				select_camera(FORWARD_CAMERA);
			}
		}
	}
}

// Desired heading to change from desired heading to motor commands
void obav_desired_heading_callback(const obstacle_avoidance::DesiredHeading desired_heading_msg) {

	obs_direction = desired_heading_msg.direction;
	obs_magnitude = desired_heading_msg.magnitude;

}

void publish_navigation_message() {
	navigation_msg.l_motor_val = left_motor;
    navigation_msg.r_motor_val = right_motor;
    navigation_msg.dest_lat = head_to_dest;
    navigation_msg.dest_long = dis_to_dest;
    navigation_msg.waypoint_id = curr_state;

    navigation_pub.publish(navigation_msg);
}

void arm_state_callback(const std_msgs::String arm_state_msg) {
	if (curr_state != ARM_PICKUP)
		return;

	if(arm_state_msg.data.compare("in_progress") == 0) {
		navigation_msg.relay_state = true;
	} else {
		navigation_msg.relay_state = false;
		curr_state = MOVE_TO_WAYPOINT;
		select_camera(FORWARD_CAMERA);
	}
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "navigation");
  ros::NodeHandle n;

  //Publisher registration
  navigation_pub = n.advertise<navigation::Navigation>("navigation", 1000);
  debug_pub = n.advertise<navigation::Debug>("debug", 1000);
  camera_select_pub = n.advertise<std_msgs::Int8>("vision/yolo2/camera_select", 1000);
  arm_pub = n.advertise<navigation::Arm>("arm", 1000);

  //Subscriber registration
  arduino_sub = n.subscribe("arduino", 1000, arduino_callback);
  arm_state_sub = n.subscribe("arm_state", 1000, arm_state_callback);
  joystick_sub = n.subscribe("joy", 1000, joystick_callback);
  obav_desired_heading_sub = n.subscribe("desired_heading", 1000, obav_desired_heading_callback);
  object_track_sub = n.subscribe("object_track", 1000, object_track_callback);
  yolo_sub = n.subscribe("vision/yolo2/detections", 1000, yolo_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()) {
  	publish_navigation_message();
  	publish_debug_message();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
