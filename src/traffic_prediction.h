#ifndef TRAFFIC_PREDICTION_H
#define TRAFFIC_PREDICTION_H

#include "helper_funcs.h"
#include <map>

enum TargetLane {
	LEFT = 0,
	CURRENT,
	RIGHT,	
};

const double kLaneWidth = 4.0;

/****************************/
/* class TrafficParticipant */
/****************************/

class TrafficParticipant {
	
	public:
	
	// member variables
	int    id;
	
	double x;
	double y;
	double vx;
	double vy;
	
	double s;
	double d;	
	double vs;
	double vd;
	double rel_d; // relative lateral position (d), 0 <= rel_d <= 1
	int    lane_index;
		
	TargetLane target_lane;
		
	
	// member functions
	TrafficParticipant(std::vector<double> & fused_vehicle, Map & map);
	~TrafficParticipant();
		
	void DetectManeuver();
	std::vector<double> GetFrenetStateInTime(double time);
	
};

/***************************/
/*      class Traffic      */
/***************************/

class Traffic {

	public:
	
	// member variables
	std::map< int, TrafficParticipant > traffic_participants;
	
	//member functions
	Traffic(std::vector< std::vector<double> > & sensor_fusion, Map & map); 
	~Traffic();
	
	

};




#endif