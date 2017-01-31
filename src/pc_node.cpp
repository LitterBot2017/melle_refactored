#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "melle_obstacle_avoidance/ObAvData.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include "melle_refactored/MellE_msg.h"
#include "melle_refactored/PC_msg.h"
#include "PID.h"
#include "PID_horz.h"

//States
#define GET_GPS_LOCK 1
#define MOVE_TO_WAYPOINT 2
#define OBSTACLE_AVOIDANCE 3
#define JOYSTICK 4
int curr_state = GET_GPS_LOCK;

//GPS_Waypoints
float lat_list [] = { 40.442172, 40.442018, 40.442122 };
float long_list [] = { -79.945352, -79.945370, -79.945629 };
int curr_ind = 0;

//GPS_Stuff
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

float curr_lat;
float curr_long;
float curr_heading;
float dest_lat = lat_list[curr_ind];
float dest_long = long_list[curr_ind];
long elapsedTime;
float dis_to_dest;
float head_to_dest;

//Diagnostic data
float batt_level;
float bin_fullness;

//Subscribers
ros::Subscriber melle_sub; 
ros::Subscriber joystick_sub;
ros::Subscriber ob_av_sub;

//Publishers
ros::Publisher base_pub;

//Publisher msgs
melle_refactored::PC_msg msg_to_send;

//Motor_controller calculator and PID
#define MAX_TURNING_SPEED 20
#define MAX_FORWARD_SPEED 50
PID turn_pid = PID(0, 10, 0.01, 0, 0, MAX_TURNING_SPEED, -1 * MAX_TURNING_SPEED);
PID_horz forward_pid = PID_horz(0, 10, 0.01, 0, 0, MAX_FORWARD_SPEED, -1 * MAX_FORWARD_SPEED);
float left_motor;
float right_motor;

void calculate_motor_speed()
{
	
	if(curr_state != GET_GPS_LOCK && curr_state != MOVE_TO_WAYPOINT)
	{
		msg_to_send.waypoint_id = 40;
		return;
	}
	else if(curr_state == GET_GPS_LOCK)
	{
		msg_to_send.waypoint_id = 50;
		left_motor = 64;
		right_motor = 64;
		return;
	}
	else if(curr_state == MOVE_TO_WAYPOINT)
	{
		msg_to_send.waypoint_id = left_motor;
		float turn_speed = turn_pid.getNewValue(curr_heading, head_to_dest, elapsedTime);
		float forward_speed = forward_pid.getNewValue(dis_to_dest, elapsedTime);
		turn_speed = (turn_speed/128) * 64;
		forward_speed = (forward_speed/128) * 64;
		left_motor =  64 + turn_speed + forward_speed;
		right_motor = 64 - turn_speed + forward_speed;
		if(left_motor > 89) {
			left_motor = 89;
		}
		if(left_motor < 39) {
			left_motor = 39;
		}
		if(right_motor > 89) {
			right_motor = 89;
		}
		if(right_motor < 39) {
			right_motor = 39;
		}
		if (dis_to_dest < 3) {
			right_motor=64;
			left_motor=64;
		}
	}
}

//GPS distance and angle calculation
double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

//MellE Callback
void melle_callback(const melle_refactored::MellE_msg msg)
{
	curr_heading = msg.heading;
	curr_long = msg.curr_long;
	curr_lat = msg.curr_lat;
	msg_to_send.waypoint_id = 10;
	elapsedTime = elapsedTime - msg.elapsed_time;
	dis_to_dest = distanceBetween(dest_lat, dest_long, curr_lat, curr_long);
	head_to_dest = courseTo(dest_lat, dest_long, curr_lat, curr_long);
	if (msg.sats != 1 && (curr_state == GET_GPS_LOCK || curr_state == MOVE_TO_WAYPOINT))
	{
		curr_state = GET_GPS_LOCK;
		msg_to_send.waypoint_id = 20;
	}
	if (msg.sats == 1 && (curr_state == GET_GPS_LOCK || curr_state == MOVE_TO_WAYPOINT))
	{
		curr_state = MOVE_TO_WAYPOINT;
		msg_to_send.waypoint_id = 30;		
	}
	calculate_motor_speed();
	if(dis_to_dest < 3)
	{
		curr_ind++;
		dest_lat = lat_list[curr_ind % 3];
		dest_long = long_list[curr_ind %3];
	}
	batt_level = msg.battery;
	bin_fullness = msg.bin_fullness;
}

//Joystick callback to switch from twist message to left and right motor commands
void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	msg_to_send.waypoint_id = 70;
	if(joy->buttons[0])
		curr_state = GET_GPS_LOCK;
	else if(joy->buttons[1])
		curr_state = OBSTACLE_AVOIDANCE;
	else if(joy->buttons[2])
		curr_state = JOYSTICK;

	if(curr_state != JOYSTICK)
	{
		return;
	}

	float turn_speed = joy->axes[3]*8;
	float forward_speed = joy->axes[0]*10;
	left_motor = 64 + forward_speed + turn_speed;
	right_motor = 64 + forward_speed - turn_speed;
}

//Ob_av_callback to change from ob_av_msg to motor commands
void ob_av_callback(const melle_obstacle_avoidance::ObAvData ob_av_msg)
{
	msg_to_send.waypoint_id = 80;
	if(curr_state != OBSTACLE_AVOIDANCE)
	{
		return;
	}
	int command = ob_av_msg.command;
	switch (command) {
	    case 0:
	      left_motor = 80;
	      right_motor = 80;
	      break;
	    case 1:
	      left_motor = 64;
	      right_motor = 80;
	      break;
	    case 2:
	      left_motor = 80;
	      right_motor = 64;
	      break;
	    case 3:
	      left_motor = 64;
	      right_motor = 64;
	      break;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_node");
  ros::NodeHandle n;
  //Publisher registration
  base_pub = n.advertise<melle_refactored::PC_msg>("PC_msg", 1000);
  //Subscriber registration
  melle_sub = n.subscribe("MellE_msg", 1000, melle_callback);
  joystick_sub = n.subscribe("joy", 1000, joystick_callback);
  ob_av_sub =n.subscribe("ob_av_msg", 1000, ob_av_callback);
  ros::Rate loop_rate(1);

  while(ros::ok())
  {
  	msg_to_send.l_motor_val = left_motor;
    msg_to_send.r_motor_val = right_motor;
    msg_to_send.dest_lat = dest_lat;
    msg_to_send.dest_long = dest_long;
    msg_to_send.waypoint_id = curr_state;
    ros::spinOnce();
    loop_rate.sleep();
    base_pub.publish(msg_to_send);
  }

  return 0;
}
