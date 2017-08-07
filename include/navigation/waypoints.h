/*
 * Waypoints.h
 *
 *  Created on: Nov 4, 2016
 *      Author: Admin
 */

#ifndef WAYPOINTS_H_
#define WAYPOINTS_H_

class Waypoint{
	public:
		float lat_val;
		float long_val;
		Waypoint(float lat_val, float long_val);
		Waypoint();
};

class Waypoints {

	public:
		Waypoints(Waypoint start, Waypoint end);
		Waypoints();
		Waypoint getStartPoint();
		Waypoint getEndPoint();
		Waypoint getNextWaypoint();
		void markWaypointDone(Waypoint done);
		void convertToMeter(Waypoint start, Waypoint end, float& xmeter, float& ymeter);
		int numOfPoints();
		Waypoint getWaypointFromIndex(int idx);
		Waypoint* uncoveredWaypoint;
	private:
		int totalPoints;
		static const float meterPerLat=111034.605288;//111034.60528834906;
		static const float meterPerLong=85393.82609;//85393.82609037454;
		static int num_of_objects;
		Waypoint wayStart;
		Waypoint wayEnd;
		int uncoveredPointer;
		int coveredPointer;
		
		Waypoint coveredWaypoint[];
		void generateWaypoints();
		void pushWaypoint(Waypoint*  arrayToPush, int pointer, Waypoint data);
};

#endif /* WAYPOINTS_H_ */
