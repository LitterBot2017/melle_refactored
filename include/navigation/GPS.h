#ifndef GPS_H_
#define GPS_H_

#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x)*(x))

class GPS {
	public:
		static double distanceBetween(double lat1, double long1, double lat2, double long2);
		static double courseTo(double lat1, double long1, double lat2, double long2);
};

#endif /* GPS_H_ */
