#ifndef HELPER_FUNCS_H
#define HELPER_FUNCS_H

#include <vector>
#include <math.h>

struct Map {

	std::vector<double> * waypoints_x;
  	std::vector<double> * waypoints_y;
  	std::vector<double> * waypoints_s;
  	std::vector<double> * waypoints_dx;
  	std::vector<double> * waypoints_dy;
};


// For converting back and forth between radians and degrees.
inline constexpr double pi()     { return M_PI; }
inline double deg2rad(double x)  { return x * pi() / 180; }
inline double rad2deg(double x)  { return x * 180 / pi(); }
inline double logistic(double x) { return 2.0 / (1 + exp(-x)) - 1.0 ; }
//inline double CalcDistanceOfTwoPoints(double x1, double y1, double x2, double y2) { return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );}
double distance(double x1, double y1, double x2, double y2);


int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);


int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);


// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, std::vector<double> maps_s, std::vector<double> maps_x, std::vector<double> maps_y);

// ------------------------------------------ my helper functions start --------------------------------------------------

// get the lane index from the d value

inline int GetLaneIndex(double d, double lane_width) { return int( (d-fmod(d,lane_width))/lane_width ) ; } 


// Transform a point from global X, Y to car frame

void TransformGlobalToCarFrame(const double pt_x, const double pt_y, double & pt_x_car_frame, double & pt_y_car_frame, const std::vector<double> & ref);


// Transform a list of points from global X, Y to car frame
void TransformGlobalToCarFrame(const std::vector<double> & ptsx, const std::vector<double> & ptsy, std::vector<double> & ptsx_car_frame, std::vector<double> & ptsy_car_frame, const std::vector<double> & ref);


void TransformCarFrameToGlobal(const double pt_x_car_frame, const double pt_y_car_frame, double & pt_x, double & pt_y , const std::vector<double> & ref);


std::vector<double> getFrenetSpeed(double x, double y, double vx, double vy, const Map & map);

// ------------------------------------------ my helper functions end --------------------------------------------------




#endif

