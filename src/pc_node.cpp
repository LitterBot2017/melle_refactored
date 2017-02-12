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
#include "melle_refactored/Debug_msg.h"
#include "downview_cam/po.h"
#include "PID.h"
#include "PID_horz.h"
using namespace std;

//States
#define GET_GPS_LOCK 1
#define MOVE_TO_WAYPOINT 2
#define OBSTACLE_AVOIDANCE 3
#define JOYSTICK 4
#define DOWNVIEW_CAM 5
int curr_state = GET_GPS_LOCK;

//GPS_Waypoints
float lat_list [] = { 40.4421768188, 40.4420509338, 40.442111969};
float long_list [] = { -79.9453964233, -79.9453735352,-79.9455795288 };
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
float speed;
int sats;

int max_speed_teleop=20;

//Diagnostic data
float batt_level;
float bin_fullness;
string downview_state;

//Debug mode
bool enable_debug_mode=false;

//Subscribers
ros::Subscriber melle_sub; 
ros::Subscriber joystick_sub;
ros::Subscriber ob_av_sub;
ros::Subscriber downview_cam_sub;

//Publishers
ros::Publisher base_pub;
ros::Publisher debug_pub;

//Publisher msgs
melle_refactored::PC_msg msg_to_send;
melle_refactored::Debug_msg debug_msg;

//Motor_controller calculator and PID
#define MAX_TURNING_SPEED 20
#define MAX_FORWARD_SPEED 50
PID turn_pid = PID(0, 10, 0.01, 0, 0, MAX_TURNING_SPEED, -1 * MAX_TURNING_SPEED);
PID_horz forward_pid = PID_horz(0, 10, 0.01, 0, 0, MAX_FORWARD_SPEED, -1 * MAX_FORWARD_SPEED);
float left_motor=64;
float right_motor=64;

//Debug message creator
void publish_debug_msg(float c_lat,float c_long, int run_time,int satellites,float gps_odom,float direction,
						int bin_diag, int batt_level_read, float l_motor, float r_motor, float d_lat, float d_long,
						int way_id, float bearing, float dist_away, float head_error, int current_state, string downview_state)
{
	debug_msg.curr_lat =c_lat;
	debug_msg.curr_long = c_long;
	debug_msg.elapsed_time = run_time;
	debug_msg.sats = satellites;
	debug_msg.speed_val = gps_odom;
	debug_msg.heading = direction;
	debug_msg.bin_fullness = bin_diag;
	debug_msg.battery = batt_level_read;
	//debug_msg.l_motor = l_motor;
	//debug_msg.r_motor = r_motor;
	debug_msg.dest_lat = d_lat;
	debug_msg.dest_long = d_long;
	debug_msg.waypoint_id = way_id;
	debug_msg.head_to_dest = bearing;
	debug_msg.dist_to_dest = dist_away;
	debug_msg.head_error = head_error;
	debug_msg.current_state = current_state;
	debug_msg.downview_state = downview_state;
}

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
		float turn_speed = turn_pid.getNewValue(curr_heading, head_to_dest, elapsedTime);
		float forward_speed = forward_pid.getNewValue(dis_to_dest, elapsedTime);
		//debug_msg.r_motor=turn_speed;
		//debug_msg.l_motor=forward_speed;
		turn_speed = turn_speed/2;
		forward_speed = forward_speed/2;
		if (abs(curr_heading - head_to_dest) < 20)
  		{

      		left_motor =  40;
  			right_motor = 40;
  		} else if (curr_heading - head_to_dest) {
      		left_motor =  40;
  			right_motor = 88;

  		} else {

      		left_motor =  88;
  			right_motor = 40;
  		}
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
  double turn_to_degrees=degrees(a2);
  return turn_to_degrees;
}

void motor_turn(float x_pos, float y_pos, float* motor_l, float* motor_r )
{
	if(x_pos-640>50)
	{
		left_motor =  64 + 4;
  		right_motor = 64 - 4;
	}
	else if(x_pos-640<50)
	{
		left_motor =  64 - 4;
  		right_motor = 64 + 4;
	}
	if(x_pos<690 && x_pos>590)
	{
		if(y_pos-250>50)
		{
			left_motor =  64 - 3;
  			right_motor = 64 - 3;
		}
		else if(y_pos-250<50)
		{
			left_motor =  64 + 3;
  			right_motor = 64 + 3;
		}
	}	
}

//Downview Camera Callback
void downview_cam_callback(const downview_cam::po msg)
{
	if(curr_state!=DOWNVIEW_CAM)
		return;
	downview_state=msg.command;
	if(msg.command.compare("not_detected")==0)
	{
		left_motor = 75;
	    right_motor = 75;
	}
	else if(msg.command.compare("detected")==0)
	{
		motor_turn(msg.x,msg.y,&left_motor,&right_motor);
	}
	else if(msg.command.compare("centered")==0)
	{
		left_motor=64;
		right_motor=64;
		curr_state=JOYSTICK;
	}
}

//MellE Callback
void melle_callback(const melle_refactored::MellE_msg msg)
{
	curr_heading = msg.heading;
	curr_long = msg.curr_long;
	curr_lat = msg.curr_lat;
	speed = msg.speed_val;
	sats=msg.sats;
	msg_to_send.waypoint_id = 10;
	elapsedTime = elapsedTime - msg.elapsed_time;
	dis_to_dest = distanceBetween(curr_lat, curr_long, dest_lat, dest_long);
	head_to_dest = courseTo(curr_lat, curr_long, dest_lat, dest_long);
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
		dest_lat = lat_list[curr_ind % 2];
		dest_long = long_list[curr_ind %2];
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
	else if(joy->buttons[3])
		curr_state=DOWNVIEW_CAM;
	else if(joy->buttons[9])
	{
		if(enable_debug_mode)
		{
			enable_debug_mode=false;
		}
		else
		{
			enable_debug_mode=true;
		}
	}

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
	right_motor =(64 + forward_speed + turn_speed);
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
	      left_motor = 80;
	      right_motor = 64;
	      break;
	    case 2:
	      left_motor = 64;
	      right_motor = 80;
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
  debug_pub = n.advertise<melle_refactored::Debug_msg>("Debug_msg", 1000);
  //Subscriber registration
  melle_sub = n.subscribe("MellE_msg",1000,melle_callback);
  joystick_sub = n.subscribe("joy",1000,joystick_callback);
  ob_av_sub =n.subscribe("ob_av_data",1000,ob_av_callback);
  downview_cam_sub =n.subscribe("down_cam_msg",1000,downview_cam_callback);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
  	msg_to_send.l_motor_val = left_motor;
    msg_to_send.r_motor_val = right_motor;
    msg_to_send.dest_lat = head_to_dest;
    msg_to_send.dest_long = dis_to_dest;
    msg_to_send.waypoint_id = curr_state;
    if(enable_debug_mode)
    {
    	publish_debug_msg(curr_lat,curr_long, elapsedTime,sats,speed,curr_heading,
						bin_fullness,batt_level, left_motor, right_motor, dest_lat, dest_long,
						curr_ind, head_to_dest, dis_to_dest, curr_heading-head_to_dest, curr_state, downview_state);
    	debug_pub.publish(debug_msg);
    }	
    ros::spinOnce();
    loop_rate.sleep();
    base_pub.publish(msg_to_send);
  }

  return 0;
}
