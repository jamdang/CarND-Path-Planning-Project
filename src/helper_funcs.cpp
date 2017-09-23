#include "helper_funcs.h"
using namespace std;

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = fabs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// ------------------------------------------ my helper functions start --------------------------------------------------




// Transform a point from global X, Y to car frame

void TransformGlobalToCarFrame(const double pt_x, const double pt_y, double & pt_x_car_frame, double & pt_y_car_frame, const vector<double> & ref){

	double ref_x = ref[0];
	double ref_y = ref[1];
	double ref_yaw = ref[2];
			
	double dx = pt_x - ref_x;
	double dy = pt_y - ref_y;
		
	pt_x_car_frame = dx * cos(ref_yaw) + dy * sin(ref_yaw);
	pt_y_car_frame = dy * cos(ref_yaw) - dx * sin(ref_yaw);
	
}

// Transform a list of points from global X, Y to car frame
void TransformGlobalToCarFrame(const vector<double> & ptsx, const vector<double> & ptsy, vector<double> & ptsx_car_frame, vector<double> & ptsy_car_frame, const vector<double> & ref){

	double pt_x;
	double pt_y;
		
	ptsx_car_frame.resize(ptsx.size());
	ptsy_car_frame.resize(ptsx.size());	
	
	for (int i = 0; i < ptsx.size(); i++) {
		TransformGlobalToCarFrame(ptsx[i], ptsy[i], ptsx_car_frame[i], ptsy_car_frame[i], ref);
	}
}


void TransformCarFrameToGlobal(const double pt_x_car_frame, const double pt_y_car_frame, double & pt_x, double & pt_y , const vector<double> & ref){

	double ref_x = ref[0];
	double ref_y = ref[1];
	double ref_yaw = ref[2];
	
	pt_x = ref_x + pt_x_car_frame * cos(ref_yaw) - pt_y_car_frame * sin(ref_yaw);
	pt_y = ref_y + pt_y_car_frame * cos(ref_yaw) + pt_x_car_frame * sin(ref_yaw);
	
}

vector<double> getFrenetSpeed(double x, double y, double vx, double vy, const Map & map)
{
	double frenet_vs = 0.0;
	double frenet_vd = 0.0;
	
	int ind_closest_wp = ClosestWaypoint(x,y, *(map.waypoints_x), *(map.waypoints_y));

	double dx = (*(map.waypoints_dx))[ind_closest_wp]; // waypoints_dx is a pointer
	double dy = (*(map.waypoints_dy))[ind_closest_wp];
	

	// find the projection of speed onto d 
	
	frenet_vd = dx * vx + dy * vy;
	
	frenet_vs = sqrt(vx * vx + vy * vy - frenet_vd * frenet_vd); 
	
	
	return {frenet_vs, frenet_vd};
	
	
}

// ------------------------------------------ my helper functions end --------------------------------------------------