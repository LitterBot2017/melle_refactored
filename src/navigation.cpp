#include "ros/ros.h"

#include "arduino_pc/Arduino.h"

#include "navigation/Navigation.h"
#include "navigation/Debug.h"
#include "navigation/Arm.h"

#include "object_tracker/BBox.h"
#include "obstacle_avoidance/DesiredHeading.h"
#include "yolo2/ImageDetections.h"
#include "yolo2/Detection.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Joy.h>

#include "GPS.h"
#include "PID.h"
#include "PID_horz.h"

using namespace std;

//States
#define GET_GPS_LOCK 1
#define MOVE_TO_WAYPOINT 2
#define JOYSTICK 3
#define OBJECT_TRACK 4

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

//Detectors
bool detected = false;

//Subscribers
ros::Subscriber arduino_sub;
ros::Subscriber joystick_sub;
ros::Subscriber obav_desired_heading_sub;
ros::Subscriber object_track_sub;
ros::Subscriber yolo_sub;

//Publishers
ros::Publisher navigation_pub;
ros::Publisher debug_pub;
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
void publish_debug_msg(float c_lat,float c_long, int run_time,int satellites,float gps_odom,float direction,
						int bin_diag, int batt_level_read, float l_motor, float r_motor, float d_lat, float d_long,
						int way_id, float bearing, float dist_away, float head_error, int current_state, string downview_state, float obs_modifier)
{
	debug_msg.curr_lat = c_lat;
	debug_msg.curr_long = c_long;
	debug_msg.elapsed_time = run_time;
	debug_msg.sats = satellites;
	debug_msg.speed_val = gps_odom;
	debug_msg.heading = direction;
	debug_msg.bin_fullness = bin_diag;
	debug_msg.battery = batt_level_read;
	debug_msg.l_motor = l_motor;
	debug_msg.r_motor = r_motor;
	debug_msg.dest_lat = d_lat;
	debug_msg.dest_long = d_long;
	debug_msg.waypoint_id = way_id;
	debug_msg.head_to_dest = bearing;
	debug_msg.dist_to_dest = dist_away;
	debug_msg.head_error = head_error;
	debug_msg.current_state = current_state;
	debug_msg.downview_state = downview_state;
	debug_msg.obs_modifier = obs_modifier;
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

void motor_turn(float x_pos, float y_pos, float* motor_l, float* motor_r) {

	if(x_pos - 640 > 50) {
		left_motor =  64 + 4;
  		right_motor = 64 - 4;
	}
	else if(x_pos - 640 < 50) {
		left_motor =  64 - 4;
  		right_motor = 64 + 4;
	}
	if(x_pos < 690 && x_pos > 590) {
		if(y_pos - 250 > 50) {
			left_motor =  64 - 3;
  			right_motor = 64 - 3;
		}
		else if(y_pos - 250 < 50) {
			left_motor =  64 + 3;
  			right_motor = 64 + 3;
		}
	}	
}

//Object Track Callback
void object_track_callback(const object_tracker::BBox msg)
{
	if(curr_state != OBJECT_TRACK)
		return;

	navigation::Arm arm_msg;
	if(detected && (msg.x < 340 && msg.x > 300) && (msg.y < 260 && msg.y > 220))
	{
		arm_msg.x = msg.x;
		arm_msg.y = msg.y;
		arm_msg.is_centered = "centered";
		navigation_msg.relay_state = true;
		left_motor = 64;
		right_motor = 64;
		curr_state = JOYSTICK;
	}
	else if(detected)
	{
		arm_msg.x = msg.x;
		arm_msg.y = msg.y;
		arm_msg.is_centered = "detected";
		motor_turn(msg.x, msg.y, &left_motor, &right_motor);
	}
	else if(!detected)
	{
		arm_msg.is_centered = "not_detected";
	}
	arm_pub.publish(arm_msg);
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
		arm_msg.pickup_state="off";
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
void yolo_callback(const yolo2::ImageDetections detection_msg)
{
	if (detection_msg.num_detections > 0) {
		detected = true;
	} else {
		detected = false;
	}
}

// Desired heading to change from desired heading to motor commands
void obav_desired_heading_callback(const obstacle_avoidance::DesiredHeading desired_heading_msg) {

	obs_direction = desired_heading_msg.direction;
	obs_magnitude = desired_heading_msg.magnitude;

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "navigation");
  ros::NodeHandle n;

  //Publisher registration
  navigation_pub = n.advertise<navigation::Navigation>("navigation", 1000);
  debug_pub = n.advertise<navigation::Debug>("debug", 1000);
  arm_pub = n.advertise<navigation::Arm>("arm", 1000);

  //Subscriber registration
  arduino_sub = n.subscribe("arduino", 1000, arduino_callback);
  joystick_sub = n.subscribe("joy", 1000, joystick_callback);
  obav_desired_heading_sub = n.subscribe("desired_heading", 1000, obav_desired_heading_callback);
  object_track_sub = n.subscribe("bbox", 1000, object_track_callback);
  yolo_sub = n.subscribe("vision/yolo2/detections",1000,yolo_callback);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
  	navigation_msg.l_motor_val = left_motor;
    navigation_msg.r_motor_val = right_motor;
    navigation_msg.dest_lat = head_to_dest;
    navigation_msg.dest_long = dis_to_dest;
    navigation_msg.waypoint_id = curr_state;
    if(enable_debug_mode)
    {
    	publish_debug_msg(curr_lat,curr_long, elapsedTime,sats,speed,curr_heading,
						bin_fullness,batt_level, left_motor, right_motor, dest_lat, dest_long,
						curr_ind, head_to_dest, dis_to_dest, curr_heading-head_to_dest, curr_state, downview_state,obs_magnitude_modifier);
    	debug_pub.publish(debug_msg);
    }

    navigation_pub.publish(navigation_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
