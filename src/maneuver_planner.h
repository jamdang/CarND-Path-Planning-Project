#ifndef MANEUVER_PLANNER_H
#define MANEUVER_PLANNER_H


#include "traffic_prediction.h"

#define INVALID -1 
#define INVALID_DOUBLE -6666.6

const double kDistanceTooFar = 50.0;
const double MPH_to_MPS   = .447;
const double kSpeedLimit  = 50 * MPH_to_MPS;


enum FSMState {

	LK = 0,
	LCL,
	LCR,
	PLCL,
	PLCR,
};
 
struct ManeuverPlan {

	int    target_lane_id;
	int    target_leading_veh_id;
	double target_speed;
	double time_to_reach_target;

};

struct EgoCar {

	double x;
	double y;
	double s;
	double d;
	double yaw;
	double spd;
	
	int current_lane_id;
	double ref_vel;
	
	ManeuverPlan maneuver;

};



void PlanManeuver(EgoCar & ego_car, const Traffic & traffic);

int  GetLeadVehId(const EgoCar & ego_car, const Traffic & traffic);

inline bool isInvalid(double x) { return (x < -6666.0); }
inline bool isInvalid(int x)    { return (x==INVALID) ; }
 
#endif