#include "ego_car.h"
#include <iostream>
using namespace std;

EgoCar::EgoCar() {

	// this->traj_cost_functions.push_back( & this->CalcTrajCostOfBuffer );
	 //TrajCostFunction traj_cost_function = & this->CalcTrajCostOfBuffer;
}




EgoCar::~EgoCar() {
}

void EgoCar::PlanPath(const Traffic  & traffic        , const Map & map, 							 									   
					  vector<double> & next_x_vals    , vector<double> & next_y_vals){
	
	PlanManeuver(traffic);
	GenerateTrajectory( traffic        , map, 							 									   
						next_x_vals    , next_y_vals);
						
}

/******************************/
/***    Maneuver Planner    ***/
/******************************/

void EgoCar::PlanManeuver(const Traffic & traffic) {

	
	this->current_lane_id = GetLaneIndex(this->d, kLaneWidth);	
	
	int leading_veh_id = GetLeadVehId(traffic);
	auto lead_veh_iter = traffic.traffic_participants.find(leading_veh_id);
	
	
	
	// temp code
	
	this->fsm_state = LK;
	
	if (lead_veh_iter != traffic.traffic_participants.end()) {
		if (lead_veh_iter->second.vs < kSpeedLimit) {
			if (this->current_lane_id > 0)
				this->fsm_state = LCL;
		}
		
	}
	
	switch (this->fsm_state) {

  	  case LK: { 
  	  
  	    this->maneuver.target_lane_id = this->current_lane_id;
  		this->maneuver.target_leading_veh_id = leading_veh_id;  		
  		this->maneuver.target_speed = INVALID_DOUBLE;
  		//this->maneuver.time_to_reach_target = 5.0;
  		//this->maneuver.target_delta_s = -30.0;
  		
  		if (isInvalid(leading_veh_id)) {
  			this->maneuver.target_speed = kSpeedLimit;
  			//this->maneuver.time_to_reach_target = INVALID_DOUBLE; 
  			//this->maneuver.target_delta_s = INVALID_DOUBLE; 
  		}		
  		
		else {	
			auto iter = traffic.traffic_participants.find(leading_veh_id);
			if (iter != traffic.traffic_participants.end())
				this->maneuver.target_speed = (iter->second).vs;		
		}
  		
  		          
        break;
  	  }
  	  
  	  case LCL: { 
  	  
  	    this->maneuver.target_lane_id = this->current_lane_id - 1;
  		this->maneuver.target_leading_veh_id = INVALID;  		
  		this->maneuver.target_speed = INVALID_DOUBLE;
  		
  		if (isInvalid(this->maneuver.target_leading_veh_id)) {
  			this->maneuver.target_speed = kSpeedLimit;

  		}		
  		
		else {	
			auto iter = traffic.traffic_participants.find(this->maneuver.target_leading_veh_id);
			if (iter != traffic.traffic_participants.end())
				this->maneuver.target_speed = (iter->second).vs;		
		}
  		
  		          
        break;
  	  }


  	  default: {
    
  	  }

	}
		
}



int  EgoCar::GetLeadVehId(const Traffic & traffic) {
	
	int lead_veh_id = INVALID;
	
	double closest_distance = 10000; // large number
	
	for (auto mapIter = traffic.traffic_participants.begin(); mapIter != traffic.traffic_participants.end(); mapIter++) {
		
		// only interested in cars on ego lane
		TrafficParticipant traffic_participant = mapIter->second;
		if (traffic_participant.lane_index != this->current_lane_id)
			continue; 
		// only interested in cars in the front
		double dist = traffic_participant.s - this->s;
		if (dist < 0)
			continue;
		// get the info of the closet car in ego lane so far
		if (dist < closest_distance) {
			lead_veh_id = traffic_participant.id;
			closest_distance = dist;
		}			
		
	}
		
	if (closest_distance > kDistanceTooFar) {
		lead_veh_id = INVALID;
	}
	
	return lead_veh_id;
	
}




/********************************/
/***    Trajectory Planner    ***/
/********************************/

void EgoCar::PrepRefPoints() {
	double ref_x   = this->x;
    double ref_y   = this->y;
	double ref_yaw = deg2rad(this->yaw);
	
	int prev_path_size = this->previous_path_x.size();
	
	if (prev_path_size < 2) {
		double prev_car_x = this->x - 1.0 * cos(deg2rad(this->yaw));
		double prev_car_y = this->y - 1.0 * sin(deg2rad(this->yaw));
		
		this->ref_ptsx.push_back(prev_car_x);
		this->ref_ptsx.push_back(this->x);
		
		this->ref_ptsy.push_back(prev_car_y);
		this->ref_ptsy.push_back(this->y);
	}
	
	else {
		// redefine reference points
		
		ref_x = this->previous_path_x[prev_path_size - 1];
		ref_y = this->previous_path_y[prev_path_size - 1];
		
		double prev_ref_x = this->previous_path_x[prev_path_size - 2];
		double prev_ref_y = this->previous_path_y[prev_path_size - 2];
		
		ref_yaw = atan2( (ref_y - prev_ref_y) , (ref_x - prev_ref_x) );
		
		this->ref_ptsx.push_back(prev_ref_x);
		this->ref_ptsx.push_back(ref_x);
		
		this->ref_ptsy.push_back(prev_ref_y);
		this->ref_ptsy.push_back(ref_y);
		
	}
	
	this->ref = {ref_x, ref_y, ref_yaw};

}


void EgoCar::GeneratePossibleGoals(const Traffic & traffic, vector< vector<double> > & goals) {
	// the maneuver plan include : 
	// int    target_lane_id;
	// int    target_leading_veh_id;
	// double target_speed;
	// double time_to_reach_target;
	
	double target_s;
	double target_d;
	double target_vs;
	
	target_d = this->maneuver.target_lane_id * kLaneWidth + kLaneWidth/2;
	
	int loop_times = 4;
	
	if (this->maneuver.target_lane_id == current_lane_id) {
		loop_times = 1;
	}
	
	for (int i = 0; i < loop_times; i++) {
		
		target_s = this->s + double(i+3) * max(1.0, this->spd);
		if (loop_times == 1) {
			target_s = this->s + 30.0; 
		}
		for(int j = -7; j < 7; j ++) {
			double dv = 1.0; // m/s 
			target_vs = this->maneuver.target_speed + double(j) * dv; 
			if (target_vs < 0 || target_vs > 1.01 * kSpeedLimit)
				continue;
			goals.push_back({target_s, target_d, target_vs});
		}
	}
					

}


void EgoCar::GeneratePossibleTrajectories(const Traffic & traffic, const Map & map, const vector< vector<double> > & goals,
								           vector< NextTrajectory > & next_trajectory_candidates) {
								           
	for (int i = 0; i < goals.size(); i++) {
		double target_s = goals[i][0];
		double target_d = goals[i][1];
		double target_vs = goals[i][2];
		
		vector <double> anchor_ptsx;
		vector <double> anchor_ptsy;
		
		for (int k = 0; k < this->ref_ptsx.size(); k++) {		
			anchor_ptsx.push_back(this->ref_ptsx[k]);
			anchor_ptsy.push_back(this->ref_ptsy[k]);
		}
								
		// push three more points to the list , each spaced 30 m --- 

		double ref_vel; 

	
		//debug
		/*
		cout << "s_pt1: " << s_pt1 << endl
			 << "ref_vel: " << ref_vel << endl
			 << "this->maneuver.target_leading_veh_id: " << this->maneuver.target_leading_veh_id << endl
			 << "this->maneuver.time_to_reach_target: " << this->maneuver.time_to_reach_target << endl
			 << "isInvalid(this->maneuver.target_leading_veh_id): " << isInvalid(this->maneuver.target_leading_veh_id) << endl;
		*/
	
		vector<double> next_pt1 = getXY(target_s     , target_d, *(map.waypoints_s), *(map.waypoints_x), *(map.waypoints_y));
		vector<double> next_pt2 = getXY(target_s + 30, target_d, *(map.waypoints_s), *(map.waypoints_x), *(map.waypoints_y));
		vector<double> next_pt3 = getXY(target_s + 60, target_d, *(map.waypoints_s), *(map.waypoints_x), *(map.waypoints_y));
	
		anchor_ptsx.push_back(next_pt1[0]);
		anchor_ptsx.push_back(next_pt2[0]);
		anchor_ptsx.push_back(next_pt3[0]);
	
		anchor_ptsy.push_back(next_pt1[1]);
		anchor_ptsy.push_back(next_pt2[1]);
		anchor_ptsy.push_back(next_pt3[1]);
	
		// transform ptsx, ptsy to car frame
	
		vector<double> anchor_ptsx_car_frame;
		vector<double> anchor_ptsy_car_frame;	
										
		TransformGlobalToCarFrame(anchor_ptsx, anchor_ptsy, anchor_ptsx_car_frame, anchor_ptsy_car_frame, this->ref);
				
		// create a spline ---
		tk::spline spln;
		spln.set_points(anchor_ptsx_car_frame, anchor_ptsy_car_frame);
				
		
				
		// 
		NextTrajectory next_traj;
		

	    
		double target_x = 30;
		double target_y = spln(target_x);
		double target_dist = sqrt(target_x * target_x + target_y * target_y);
	
	
		double x_add_on = 0;
		double y_add_on;
	
		double mod_vel = this->ref_vel;
	
		//cout << "this->spd" << this->spd << endl;
				
		//cout << "mod_vel" << mod_vel << endl;
		
		//double Max_dv = kMaxAccel * 0.02;
		
		for (int j = 0; j < kComparePathSize; j++) { 
	
			double pt_x;
			double pt_y;
			
		
			if (mod_vel < target_vs - kMaxAccel * 0.02 * 1.1) {
				mod_vel += kMaxAccel * 0.02;
			}
			else if (mod_vel > target_vs + kMaxAccel * 0.02 * 1.1) {
				mod_vel -= kMaxAccel * 0.02;
			}
			else {
				mod_vel = target_vs;
			}
		
			//double N = target_dist / (0.02 * mod_vel); 
		
			x_add_on += target_x / target_dist * 0.02 * mod_vel ;
			y_add_on  = spln(x_add_on);

			TransformCarFrameToGlobal(x_add_on, y_add_on, pt_x, pt_y, this->ref);
		
			next_traj.x_vals.push_back(pt_x);
			next_traj.y_vals.push_back(pt_y);
			next_traj.vel_vals.push_back(mod_vel);
			
			double theta = this->ref[2]; // ref_yaw
			if (j > 0) {
				theta = atan2( (pt_y - next_traj.y_vals[j-1]) , (pt_x - next_traj.x_vals[j-1]) );
			}			
			//if (theta < 0) {
			//	theta += 2 * M_PI; 
			//}
			vector<double> frenet_state = getFrenet(pt_x, pt_y, theta, *(map.waypoints_x), *(map.waypoints_y));
			next_traj.s_vals.push_back(frenet_state[0]);
			next_traj.d_vals.push_back(frenet_state[1]);
			
		}
		
		next_trajectory_candidates.push_back(next_traj);
		
		
		}
									           
								           
}


NextTrajectory EgoCar::FindBestTrajectory(const Traffic & traffic, const vector< NextTrajectory > & next_trajectory_candidates) {

	NextTrajectory best_next_traj;
	double lowest_cost = 99.9;
	
	for (int i = 0; i < next_trajectory_candidates.size(); i++) {
		double cost = CalcTrajCost(traffic, next_trajectory_candidates[i]);
		if (cost < lowest_cost) {
			best_next_traj = next_trajectory_candidates[i];
			lowest_cost = cost;
		}
	
	}
	
	return best_next_traj;

}

double EgoCar::CalcTrajCost(const Traffic & traffic, const NextTrajectory & next_traj) {
	
	double weight_buffer    = 1.0;
	double weight_speed     = 1.0;
	double weight_collision = 100.0;
	double weight_lead_buffer = 4.0;
	
	double weight_sum    = weight_buffer      + 
						   weight_speed       + 
						   weight_collision   + 
						   weight_lead_buffer ;
		
	//debug
	cout << "CalcTrajCostOfBuffer:     " << CalcTrajCostOfBuffer(traffic, next_traj)     << endl 
		 << "CalcTrajCostOfSpeed:      " << CalcTrajCostOfSpeed(traffic, next_traj)      << endl
		 << "CalcTrajCostOfCollison:   " << CalcTrajCostOfCollison(traffic, next_traj)   << endl
		 << "CalcTrajCostOfFollowDist: " << CalcTrajCostOfFollowDist(traffic, next_traj) << endl;
	
	
	return ( CalcTrajCostOfBuffer     (traffic, next_traj) * weight_buffer     + 
			 CalcTrajCostOfSpeed      (traffic, next_traj) * weight_speed      + 
			 CalcTrajCostOfCollison   (traffic, next_traj) * weight_collision  + 
			 CalcTrajCostOfFollowDist (traffic, next_traj) * weight_lead_buffer ) / weight_sum;
			 


}

double EgoCar::CalcTrajCostOfBuffer(const Traffic & traffic, const NextTrajectory & next_traj) {
	
	double closest_dist = CalcNearestDistToTraffic(traffic, next_traj);
	
	return logistic(2 * kVehicleRadius / closest_dist);
}


double EgoCar::CalcTrajCostOfFollowDist(const Traffic & traffic, const NextTrajectory & next_traj) {	
	
	double cost = 0.0;
			
	if (!isInvalid(this->maneuver.target_leading_veh_id) && this->fsm_state == LK) {	
			
			auto lead_iter = traffic.traffic_participants.find(this->maneuver.target_leading_veh_id);
			if (lead_iter != traffic.traffic_participants.end()) {
				double closest_dist = CalcNearestDistToVehicle(lead_iter->second, next_traj);	
				
				//debug 
				cout << "closest_dist within the next " << kComparePathSize * 0.02 << " sec: " << closest_dist << endl;
				for (int i = 0; i < next_traj.vel_vals.size(); i+=22) {
					cout << "next_traj.vel_vals[" << i << "]: " << next_traj.vel_vals[i] << endl;
					cout << "next_traj.s_vals[" << i << "]: " << next_traj.s_vals[i] << endl;
				}
				
				
				const double kMinSafeFollowDist = 9.0;
				const double kMaxFollowDist = 80.0;
				double ideal_dist = 30.0;
				
				if (closest_dist < kMinSafeFollowDist) {
					cost = 1.0;
				}
				else if (closest_dist < ideal_dist) {
					cost = (ideal_dist - closest_dist)/(ideal_dist - kMinSafeFollowDist);
				}
				else if (closest_dist < kMaxFollowDist) {
					cost = (closest_dist - ideal_dist) / (kMaxFollowDist - ideal_dist);
				}
				else {
					cost = 1.0;
				}								
			}
	}
	
	return cost; 
	
	//return logistic(2 * kVehicleRadius / closest_dist);
}


double EgoCar::CalcNearestDistToTraffic(const Traffic & traffic, const NextTrajectory & next_traj) {
	
	double closest_dist = 999.9;
	
	for (auto veh_iter = traffic.traffic_participants.begin(); veh_iter != traffic.traffic_participants.end(); veh_iter++) {
		double dist = CalcNearestDistToVehicle(veh_iter->second, next_traj);
		if (dist < closest_dist) {
			
			closest_dist = dist;
		}
	}
	
	return closest_dist;
	
}

double EgoCar::CalcNearestDistToVehicle(const TrafficParticipant & vehicle, const NextTrajectory & next_traj) {
	
	TrafficParticipant veh = vehicle;
	
	int traj_size = next_traj.x_vals.size();
	double t0 = this->previous_path_x.size() * 0.02 ; 
	
	double nearest_dist = 999.9;
	
	for (int i = 0; i < traj_size; i++) {
		
		double time = t0 + double(i) * 0.02;
		vector<double> frenet_state_veh  = veh.GetFrenetStateInTime(time);
		
		double dist = distance(frenet_state_veh[0], frenet_state_veh[1], 
							   next_traj.s_vals[i], next_traj.d_vals[i]);
		
		if (dist < nearest_dist) 
			nearest_dist = dist;
		
	
	}
	
	return nearest_dist;

}


double CalcCostOfSpeed(double speed) {
	
	double cost = 1.0;
	double ideal_speed = kSpeedLimit - kBufferSpeed; 
	
	if (speed < ideal_speed) {
		cost = kStopCost * (ideal_speed - speed) / ideal_speed; 
	}	
	else if (speed < kSpeedLimit) {
		cost = (speed - ideal_speed) / kBufferSpeed;
	}
	
	return cost;
	
}

double EgoCar::CalcTrajCostOfSpeed(const Traffic & traffic, const NextTrajectory & next_traj) {

	double mean_cost_of_speed;
	double sum_of_cost = 0.0;
	
	for (int i = 0; i < next_traj.vel_vals.size(); i++) {
		sum_of_cost += CalcCostOfSpeed(next_traj.vel_vals[i]);
	}
	
	mean_cost_of_speed = sum_of_cost / next_traj.vel_vals.size();	
	
	return mean_cost_of_speed;

}


double EgoCar::CalcTrajCostOfCollison(const Traffic & traffic, const NextTrajectory & next_traj) {
	
	double	nearest = CalcNearestDistToTraffic(traffic, next_traj);
    if (nearest < 2*kVehicleRadius)
    	return 1.0;
    else
    	return 0.0;
	
	}

/*************************************************************************************************************/

void EgoCar::GenerateTrajectory( const Traffic        & traffic        , const Map            & map, 						    		 	   									 
								       vector<double> & next_x_vals    ,       vector<double> & next_y_vals)
{

	/* *********** 0. prepare reference points ********** */    
	// reference points are the (two) starting points of a list of anchor points from which is generated the new, add-on planned trajectory for this time step (in global coordinate)
	

	
	// ref_ptsx, ref_ptsy, represents the (two) reference points, which would mostly be the end two points of previous path
	// ref_ptsx = {prev_ref_x, ref_ptsx}
	// ref_ptsy = {prev_ref_y, ref_ptsy}
	// ref = {ref_x, ref_y, ref_yaw}
	
	PrepRefPoints();
	
	
	/* *********** 1. generate alternative goals ********** */
		
	// from this->maneuver, i.e., the planned maneuver, generate a set of possible goal points
	
	vector< vector<double> > goals;
	// each goal contains {goal_s, goal_d, goal_vs}
	GeneratePossibleGoals(traffic, goals); 
	
	//debug
	/*
	cout << "goals size: " << goals.size() << endl;
	for(int i = 0; i < goals.size(); i++) {
		cout << "goal " << i << " (target_s, target_d, target_vs): " << endl;
		for (int j = 0; j < goals[i].size(); j++) {
			cout << goals[i][j] << ", " ;
		}
		cout << endl;
	}
	*/
	
	/* *********** 2. generate possible trajectories *********** */
	// for each goal generate 1) a list of anchor points starting with the refence points 2) a spline from the anchor points 3) a trail of trajectory points from the spline
	
	vector< NextTrajectory > next_trajectory_candidates;
	
	GeneratePossibleTrajectories(traffic, map, goals,
								 next_trajectory_candidates);


    /* *********** 3. find the best trajectory *********** */
	
	
	
	NextTrajectory best_next_traj = FindBestTrajectory(traffic, next_trajectory_candidates);	
	
	
	
	/* ******* push best next trajectory to the (filtered) next traj ********* */
	int prev_path_size = this->previous_path_x.size();
	
	// debug
	cout << "previous path size: " << prev_path_size << endl;	
	cout << "follow dist: "; 
	auto lead_iter = traffic.traffic_participants.find(this->maneuver.target_leading_veh_id);
	if (lead_iter != traffic.traffic_participants.end()) {
		cout << "follow distance: " << lead_iter->second.s - this->s << endl
			 << "ahead veh speed: " << lead_iter->second.vs ;
	}
	else {
		cout << "follow distance invalid" ;
	}
	
	
	
	// push the previous (uncovered) path to the next path vector ---
	for (int i = 0; i < prev_path_size; i++) {
		next_x_vals.push_back(this->previous_path_x[i]);
		next_y_vals.push_back(this->previous_path_y[i]);
	}
	// push the new/best next trajectory to the next path vector
	for (int i = 0; i < kPathSize - prev_path_size; i ++ ) {
		next_x_vals.push_back(best_next_traj.x_vals[i]);
		next_y_vals.push_back(best_next_traj.y_vals[i]);
				
		this->ref_vel = best_next_traj.vel_vals[i];
	
	}
    
    //this->ref_vel = mod_vel;
    
    //debug
    cout << endl 
    	 << "this->maneuver.target_speed: " << this->maneuver.target_speed << endl
		 << "                    ref_vel: " << this->ref_vel << endl 
		 << "                  ego speed: " << this->spd << endl
		 << "          deg2rad(this->yaw):" << this->yaw << endl
		 << "                      ego s: " << this->s   << endl
		 << "--------------------------------------------------------------------------------" << endl;

          	
}
