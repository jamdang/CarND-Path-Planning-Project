#ifndef EGO_CAR_H
#define EGO_CAR_H


#include "traffic_prediction.h"
#include "helper_funcs.h"
#include "spline.h"


#define INVALID -1 
#define INVALID_DOUBLE -6666.6

const double kDistanceTooFar = 60.0;
const double MPH_to_MPS   = .447;
const double kSpeedLimit  = 50.0 * MPH_to_MPS;
const double kMaxAccel    = 5.0; // m/s^2
const int    kPathSize    = 50;
const int    kComparePathSize = 200; // 100 steps, equivalent to 100*0.02 = 2s (in the future)
const double kVehicleRadius = 1.5;
const double kBufferSpeed = 2; // m/s 
const double kStopCost = 0.8;


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
	double target_delta_s;
	double target_speed;
	double time_to_reach_target;

};

struct NextTrajectory {

	std::vector <double> x_vals;
	std::vector <double> y_vals;
	std::vector <double> vel_vals;
	std::vector <double> s_vals;
	std::vector <double> d_vals;

};



class EgoCar {

//typedef double (EgoCar::*TrajCostFunction)(const Traffic & , const NextTrajectory & ) ;

private:
	FSMState fsm_state;
	
	void PrepRefPoints();
	void GeneratePossibleGoals(const Traffic & traffic, std::vector< std::vector<double> > & goals);
	void GeneratePossibleTrajectories(const Traffic & traffic, const Map & map, const std::vector< std::vector<double> > & goals,
								            std::vector< NextTrajectory > & next_trajectory_candidates);
	
	NextTrajectory FindBestTrajectory(const Traffic & traffic, const std::vector< NextTrajectory > & next_trajectory_candidates);
	double CalcTrajCost(const Traffic & traffic, const NextTrajectory & next_traj);
	double CalcTrajCostOfBuffer(const Traffic & traffic, const NextTrajectory & next_traj);
	double CalcNearestDistToVehicle(const TrafficParticipant & veh, const NextTrajectory & next_traj);
	double CalcTrajCostOfSpeed(const Traffic & traffic, const NextTrajectory & next_traj);
	double CalcTrajCostOfCollison(const Traffic & traffic, const NextTrajectory & next_traj);
	
	double CalcNearestDistToTraffic(const Traffic & traffic, const NextTrajectory & next_traj) ;

	double CalcTrajCostOfFollowDist(const Traffic & traffic, const NextTrajectory & next_traj);
	
	//std::vector< TrajCostFunction > traj_cost_functions;
	
public:

	double x;
	double y;
	double s;
	double d;
	double yaw;
	double spd;
	
	int current_lane_id;
	double ref_vel;
	
	std::vector<double> ref_ptsx;
	std::vector<double> ref_ptsy;
	std::vector<double> ref; 
	
	std::vector <double> previous_path_x;
	std::vector <double> previous_path_y;
	
	ManeuverPlan maneuver;
	
	EgoCar();
	~EgoCar();
	
	void PlanPath(const Traffic       & traffic        , const Map           & map, 							 									   
				  std::vector<double> & next_x_vals    , std::vector<double> & next_y_vals);
	
	void PlanManeuver(const Traffic & traffic);

    int  GetLeadVehId(const Traffic & traffic);
    
	void GenerateTrajectory( const Traffic             & traffic        , const Map & map, 							 									   
								   std::vector<double> & next_x_vals    ,       std::vector<double> & next_y_vals);

};




inline bool isInvalid(double x) { return (x < -6666.0); }
inline bool isInvalid(int x)    { return (x==INVALID) ; }

















 
#endif