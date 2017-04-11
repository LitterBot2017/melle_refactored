/*
 * waypoints.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: Admin
 */

#include "waypoints.h"
#include "math.h"
//#include "tgmath.h"
#include "malloc.h"
#include <iostream>
#include <iomanip>
using namespace std;

Waypoint::Waypoint(float lat_val, float long_val)
{
	this->lat_val = lat_val;
	this->long_val = long_val;
}

Waypoint::Waypoint()
{
	cout << "here3" << endl;	
}

Waypoints::Waypoints()
{
}

Waypoints::Waypoints(Waypoint start, Waypoint end)
{
	this->wayStart=start;
	this->wayEnd=end;
	generateWaypoints();
	cout << "here4" << endl;
}

void Waypoints::generateWaypoints()
{
	float totalXDist;
	float totalYDist;
	convertToMeter(wayStart,wayEnd,totalXDist, totalYDist);
	int xGrid=fabs(floor(totalXDist/3));
	xGrid = xGrid == 0 ? 1 : xGrid;
	int yGrid=fabs(floor(totalYDist/3));
	yGrid = yGrid == 0 ? 1 : yGrid;
	this->uncoveredWaypoint = new Waypoint [xGrid*yGrid];
	//this->coveredWaypoint [xGrid*yGrid];
	this->totalPoints=xGrid*yGrid;
	this->coveredPointer=0;
	this->uncoveredPointer=0;
	cout<<"totalYDist"<<totalYDist<<endl;
	cout<<"yGrid"<<yGrid<<endl;
	cout<<"meterPerLong"<<meterPerLong<<endl;
	cout<<"totalXDist"<<totalXDist<<endl;
	cout<<"xGrid"<<xGrid<<endl;
	cout<<"meterPerLat"<<meterPerLat<<endl;

	float oneLatStep=(totalXDist/xGrid)/meterPerLat;
	float oneLongStep=(totalYDist/yGrid)/meterPerLong;
	cout<<"Onelatstep"<<oneLatStep<<endl;
	cout<<"Onelongstep"<<oneLongStep<<endl;
	Waypoint nextWay;
	float lastY=wayStart.long_val;
	bool addOrMinus=false;
	for(int i=0;i<xGrid;i++)
	{
		float newXPoint=(i*oneLatStep)+wayStart.lat_val;
		if(addOrMinus==false)
			addOrMinus=true;
		else
			addOrMinus=false;
		float newYPoint;
		for(int j=0;j<yGrid;j++)
		{

			if(addOrMinus)
				newYPoint=(j*oneLongStep)+lastY;
			else
				newYPoint=-(j*oneLongStep)+lastY;
			nextWay.lat_val=newXPoint;
			nextWay.long_val=newYPoint;
			this->uncoveredWaypoint[(i*yGrid)+j]=nextWay;

		}

		lastY=newYPoint;
	}
	for (int i = 0;  i < this->numOfPoints(); i++) {
		cout << i << " = (" << this->getWaypointFromIndex(i).lat_val << ", " << this->getWaypointFromIndex(i).long_val << ")" << endl;
	}
	cout<<"Here" << endl;

}

Waypoint Waypoints::getWaypointFromIndex(int idx)
{
	cout << "here" << idx <<  endl;
	return uncoveredWaypoint[idx%totalPoints];
}

int Waypoints::numOfPoints()
{
//	cout<<std::fixed<<std::setprecision(12);
//	cout<<this->uncoveredWaypoint[5].lat_val;
//	cout<<",";
//	cout<<this->uncoveredWaypoint[5].long_val;
//	cout<<endl;
	return totalPoints;
}

Waypoint Waypoints::getNextWaypoint()
{

	if(this->uncoveredPointer<numOfPoints())
	{
		Waypoint next=uncoveredWaypoint[this->uncoveredPointer];
		this->uncoveredPointer+=1;
		return next;
	}
	Waypoint error;
	error.lat_val=-1;
	error.long_val=-1;
	return error;
}

void Waypoints::markWaypointDone(Waypoint done)
{
	coveredWaypoint[coveredPointer]=done;
	this->coveredPointer+=1;
}

Waypoint Waypoints::getStartPoint()
{
	return this->wayStart;
}

Waypoint Waypoints::getEndPoint()
{
	return this->wayEnd;
}
void Waypoints::convertToMeter(Waypoint start, Waypoint end, float& xmeter, float& ymeter)
{
	float a = (40.442072-40.442222) * 111035;
	float b = (-79.945515+79.945563) * 85393.8;
    cout << "a" << a << endl;
    cout << "b" << b << endl;
	float start_lat=wayStart.lat_val;
	float start_long=wayStart.long_val;
	float end_lat=wayEnd.lat_val;
	float end_long=wayEnd.long_val;
	xmeter=(end_lat-start_lat)*meterPerLat;
    ymeter=(end_long-start_long)*meterPerLong;
    cout << "meterPerLong" << meterPerLong << endl;
    cout << "meterPerLat" << meterPerLat << endl;
    cout << "end_lat" << end_lat << endl;
    cout << "end_long" << end_long << endl;
    cout << "start_lat" << start_lat << endl;
    cout << "start_long" << start_long << endl;
    cout << "xmeter" << xmeter << endl;
    cout << "ymeter" << ymeter << endl;
}

void Waypoints::pushWaypoint(Waypoint* array,int pointer, Waypoint data)
{
	array[pointer]=data;
}

