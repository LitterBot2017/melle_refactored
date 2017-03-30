#include "ros/ros.h"

#include "navigation/Arduino.h"
#include "navigation/Navigation.h"
#include "navigation/Debug.h"
#include "navigation/Arm.h"

#include "object_tracker/BBox.h"
#include "obstacle_avoidance/ObstacleHeading.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Joy.h>

#include "GPS.h"
#include "Motor.h"
#include "Visual_Servo.h"


using namespace std;

// States
#define GET_GPS_LOCK 1
#define MOVE_TO_WAYPOINT 2
#define JOYSTICK 3
#define LITTER_PICKUP 4
#define ARM_PICKUP 8

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
float dis_to_dest=1000;
float head_to_dest=1000;
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
ros::Subscriber obstacle_heading_sub;
ros::Subscriber object_track_sub;

//Publishers
ros::Publisher navigation_pub;
ros::Publisher debug_pub;
ros::Publisher arm_pub;

//Publisher msgs
navigation::Navigation navigation_msg;
navigation::Debug debug_msg;
navigation::Arm arm_msg;

//Motor initial state
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

//Object Track Callback
void object_track_callback(const object_tracker::BBox msg) {

	bool detection = msg.detection;
	bool isDownServo = msg.down_servo;
	bool ready_for_pickup = msg.ready_for_pickup;
	if (curr_state == ARM_PICKUP) {
		navigation::Arm arm_msg;
		arm_msg.x = msg.x;
		arm_msg.y = msg.y;
		arm_msg.is_centered = "centered";
		navigation_msg.relay_state = true;
		Motor::motor_stop(&left_motor,&right_motor);
		arm_pub.publish(arm_msg);
		return;
	}
	if (curr_state != LITTER_PICKUP && detection && curr_state != JOYSTICK)
		curr_state = LITTER_PICKUP;
	if (curr_state == JOYSTICK)
		return;
	if (ready_for_pickup)
	{
		curr_state = ARM_PICKUP;
		Motor::motor_stop(&left_motor,&right_motor);
		return;
	}
	if(detection && !isDownServo)
	{
		float visual_servo_dist = Visual_Servo::calculate_distance(msg.x,msg.y,msg.x_center,msg.y_center);
		float visual_servo_angle = Visual_Servo::calculate_angle(msg.x,msg.y,msg.x_center,msg.y_center,curr_heading);
		dis_to_dest = visual_servo_dist;
		head_to_dest = visual_servo_angle;
		downview_state = "Forward servo";
		Motor::motor_turn(msg.x,msg.y,msg.x_center,msg.y_center,&left_motor,&right_motor);
		// Motor::motor_speed_visual_servo(visual_servo_dist, 
		//							 						   curr_heading, visual_servo_angle, 
		//							 						   &left_motor,&right_motor,elapsedTime);
	}
	else if(!detection && isDownServo)
	{
		Motor::move_forward_blind(&left_motor, &right_motor);
		downview_state = "Switch cam";
	}
	else if(detection && isDownServo)
	{
		float visual_servo_dist = Visual_Servo::calculate_distance(msg.x,msg.y,msg.x_center,msg.y_center);
		float visual_servo_angle = Visual_Servo::calculate_angle(msg.x,msg.y,msg.x_center,msg.y_center,curr_heading);
		dis_to_dest = visual_servo_dist;
		head_to_dest = visual_servo_angle;		
		downview_state = "Downward servo";
		Motor::motor_turn(msg.x,msg.y,msg.x_center,msg.y_center,&left_motor,&right_motor);
							//Motor::motor_speed_visual_servo(visual_servo_dist, 
							//		 						   curr_heading, visual_servo_angle, 
							//		 						   &left_motor,&right_motor,elapsedTime);
	}
	else if(curr_state == LITTER_PICKUP && !detection && !isDownServo)
	{
		curr_state = GET_GPS_LOCK;
	}
}

// Arduino Callback
void arduino_callback(const navigation::Arduino arduino_msg)
{
	curr_heading = (arduino_msg.heading);
	curr_long = arduino_msg.curr_long;
	curr_lat = arduino_msg.curr_lat;
	speed = arduino_msg.speed_val;
	sats = arduino_msg.sats;
	
	elapsedTime = elapsedTime - arduino_msg.elapsed_time;
	
	dis_to_dest = GPS::distanceBetween(curr_lat, curr_long, dest_lat, dest_long);
	head_to_dest = GPS::courseTo(curr_lat, curr_long, dest_lat, dest_long);//-((obs_magnitude/obs_magnitude_modifier)*(obs_direction));

	if (sats != 1 && (curr_state == GET_GPS_LOCK || curr_state == MOVE_TO_WAYPOINT)) {
		curr_state = GET_GPS_LOCK;
		Motor::motor_stop(&left_motor,&right_motor);
	}
	if (sats == 1 && (curr_state == GET_GPS_LOCK || curr_state == MOVE_TO_WAYPOINT)) {
		curr_state = MOVE_TO_WAYPOINT;	
		Motor::motor_speed_navigation(dis_to_dest,curr_heading,head_to_dest,&left_motor,&right_motor,elapsedTime);
	}

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
	} else if(joy->buttons[2]) {
		curr_state = JOYSTICK;
	} else if(joy->buttons[9]) {
		enable_debug_mode = !enable_debug_mode;
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

	Motor::motor_speed_joystick(joy->axes[2],joy->axes[1],max_speed_teleop,&left_motor,&right_motor);	
}

// Desired heading to change from desired heading to motor commands
void obstacle_heading_callback(const obstacle_avoidance::ObstacleHeading obstacle_heading_msg) 
{
	obs_direction = obstacle_heading_msg.direction;
	obs_magnitude = obstacle_heading_msg.magnitude;
}

void publish_navigation_message() {
	navigation_msg.l_motor_val = left_motor;
    navigation_msg.r_motor_val = right_motor;
    navigation_msg.dest_lat = head_to_dest;
    navigation_msg.dest_long = dis_to_dest;
    navigation_msg.waypoint_id = curr_ind;
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
	}
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
  arm_state_sub = n.subscribe("arm_state", 1000, arm_state_callback);
  joystick_sub = n.subscribe("joy", 1000, joystick_callback);
  obstacle_heading_sub = n.subscribe("obstacle_heading", 1000, obstacle_heading_callback);
  object_track_sub = n.subscribe("bbox", 1000, object_track_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()) {
  	publish_navigation_message();
  	publish_debug_message();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
