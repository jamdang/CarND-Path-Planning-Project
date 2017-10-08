#include "ego_car.h"
#include <iostream>
using namespace std;

EgoCar::EgoCar() {

	// this->traj_cost_functions.push_back( & this->CalcTrajCostOfBuffer );
	 //TrajCostFunction traj_cost_function = & this->CalcTrajCostOfBuffer;
	 
	this->ref_vel = 0.0;
	this->fsm_state = LK;
	this->maneuver.target_lane_id = INVALID;
	
	Cost cost;
	cost.value   = INVALID_DOUBLE;
	cost.visited = false;
	this->cost_array.push_back(cost); // for lane 0
	this->cost_array.push_back(cost); // for lane 1
	this->cost_array.push_back(cost); // for lane 2
}




EgoCar::~EgoCar() {
}

void EgoCar::PlanPath(const Traffic  & traffic        , const Map & map, 							 									   
					  vector<double> & next_x_vals    , vector<double> & next_y_vals){
	
	PlanManeuver(traffic);
	GenerateTrajectory( traffic        , map, 							 									   
						next_x_vals    , next_y_vals);
						
}



int GetLaneShift(const TargetLane tgt_lane) {
	
	int lane_shift = 0;
	
	switch (tgt_lane) {
	  
	  case LEFT: {	  
	    lane_shift = -1;
	    break;
	  }

	  case RIGHT: {	  
	    lane_shift = 1;
	    break;
	  }
	  
	  default : {
	    break;  	
	  }
	}
	
	return lane_shift;
	
}


TargetLane GetTargetLane(FSMState state) {

	TargetLane tgt_lane = CURRENT;
	
	switch (state) {
	  
	  case LK: {
	  	tgt_lane = CURRENT;  
	  	break;
	  }
	  case LCL: {
	  	tgt_lane = LEFT;
	  	break;
	  }
	  case LCR: {
	  	tgt_lane = RIGHT;
	  	break;
	  }
	  default: {
	  	break;
	  }	
	}
	
	return tgt_lane;
}


/************************************************************************************************************************/
/***                                       ***    Maneuver Planner    ***                                             ***/
/************************************************************************************************************************/

double EgoCar::GetTargetLnId(FSMState state) {

	TargetLane tgt_lane = GetTargetLane(state) ;
	int lane_shift = GetLaneShift(tgt_lane);
	return (this->current_lane_id + lane_shift);

}


vector<FSMState> EgoCar::GetSuccessorStates() {
	
	vector<FSMState> successor_states = {LK, LCL, LCR};
	
	if (this->current_lane_id == 0 || this->fsm_state == LCR) {
		successor_states.erase( std::remove( successor_states.begin(), successor_states.end(), LCL ), successor_states.end() );
	}
	
	if (this->current_lane_id == 2 || this->fsm_state == LCL) {
		successor_states.erase( std::remove( successor_states.begin(), successor_states.end(), LCR ), successor_states.end() );
	}
	
	return successor_states;

}


void EgoCar::TransitManeuver(const Traffic & traffic) { 
	
	//int leading_veh_id = GetLeadVehId(traffic, CURRENT);
	//auto lead_veh_iter = traffic.traffic_participants.find(leading_veh_id);
	
	vector<FSMState> successor_states = GetSuccessorStates();	
	
	
	double lowest_cost = 999999.9;
	FSMState best_state = NONE;
	double fltr_coeff  = 0.1;
	
	for (int i = 0; i < successor_states.size(); i++) {
		
		double cost = CalcManeuverCost(successor_states[i], traffic);
		
		double target_lane_id = GetTargetLnId(successor_states[i]);
		
		if (!isInvalid (cost_array[target_lane_id].value) ) {
			cost = fltr_coeff * cost + (1 - fltr_coeff) * cost_array[target_lane_id].value; 
		}
		cost_array[target_lane_id].value   = cost;	
		cost_array[target_lane_id].visited = true;
		
		if (cost < lowest_cost) {
			lowest_cost = cost;
			best_state = successor_states[i];
		}
	}
	
	for (int i = 0; i < cost_array.size(); i++) {
		if (!cost_array[i].visited) {
			cost_array[i].value = INVALID_DOUBLE;			
		}
		cost_array[i].visited = false;
		
		//debug
		cout << "lane " << i << " cost: " << cost_array[i].value << endl;
	}
	
	// update/transit fsm state
	this->fsm_state = best_state;
	
		
	//cout << "this->fsm_state: " << this->fsm_state << endl;
	/* 
	this->fsm_state = LK;
	
	if (lead_veh_iter != traffic.traffic_participants.end()) {
		if (lead_veh_iter->second.vs < kSpeedLimit) {
			if (this->current_lane_id > 0)
				this->fsm_state = LCL;
		}
		
	}
	*/ 

}

double CalcManeuverCostOfLnChange(FSMState state) {
	
	double cost = 0.0;
	
	switch (state) {
	
	  case LK : {
	  	cost = 0.0;
	  	break;
	  }
	  
	  case LCL : {
	  	cost = 0.05;
	  	break;
	  }
	  
	  case LCR : {
	  	cost = 0.05;
	  	break;
	  }	
	  default : {
	  	break;
	  }
	}
	
	return cost;

}

double EgoCar::CalcManeuverCost(FSMState state, const Traffic & traffic) {
	
	double weight_buffer              = 10.0;
	double weight_buffer_same_lane    = 100.0;
	double weight_speed               = 800.0;
	double weight_collision           = 800.0;
	double weight_change_ln           = 2.0;
	double weight_braking             = 100.0;
	
	//double weight_lead_buffer = 4.0;
	
	double weight_sum    = weight_buffer           + 
						   weight_buffer_same_lane +
						   weight_speed            + 
						   weight_collision        +
						   weight_change_ln        +
						   weight_braking          ;
	
	FrenetTrajectory traj = GetFrenetTraj(state, traffic);	
	
	//debug	
	cout << "state: " << state << endl;
	cout << "CalcManeuverCostOfBuffer           :   " << CalcManeuverCostOfBuffer(traffic, traj)           << endl 
		 << "CalcManeuverCostOfBuffer same lane :   " << CalcManeuverCostOfBuffer(traffic, traj, true)     << endl 
		 << "CalcManeuverCostOfSpeed            :   " << CalcManeuverCostOfSpeed(traffic, traj)            << endl
		 << "CalcManeuverCostOfCollison         :   " << CalcManeuverCostOfCollison(traffic, traj)   	   << endl
		 << "CalcManeuverCostOfLnChange         :   " << CalcManeuverCostOfLnChange(state)          	   << endl 
		 << "CalcManeuverCostOfBraking          :   " << CalcManeuverCostOfBraking(traffic, traj)  	  	   << endl ;
	/*	 	
	cout << "traj: " << endl;
	for(int i = 0; i < traj.time.size(); i+=5) {
		cout << "time: " << traj.time[i] 
			 << " s: " << traj.s_vals[i]
			 << " d: " << traj.d_vals[i] 
			 << " v: " << traj.vel_vals[i] << endl;	
	}
	*/
	// debug ends

	if (isInvalid(this->maneuver.target_lane_id)) {
		this->maneuver.target_lane_id = this->current_lane_id;
	}
	
	double prefer_coeff = 1.0;
	//if (state == this->fsm_state )
	if (GetTargetLnId(state) == this->maneuver.target_lane_id)
		prefer_coeff = 0.6;	
	
	double total_cost =    ( CalcManeuverCostOfBuffer     (traffic, traj)       * weight_buffer             + 
							 CalcManeuverCostOfBuffer     (traffic, traj, true) * weight_buffer_same_lane   +
							 CalcManeuverCostOfSpeed      (traffic, traj)       * weight_speed              + 
							 CalcManeuverCostOfCollison   (traffic, traj)       * weight_collision          +
							 CalcManeuverCostOfLnChange   (state)               * weight_change_ln          +
							 CalcManeuverCostOfBraking    (traffic, traj)       * weight_braking            ) / weight_sum ;
	
	//debug
	cout << "total_cost                         :   " << total_cost << endl; 
	
	total_cost *= prefer_coeff;	
	return total_cost;
	
}



double EgoCar::CalcManeuverCostOfBuffer(const Traffic & traffic, const FrenetTrajectory & traj, bool same_lane) {

	double closest_dist = CalcNearestDistToTraffic(traffic, traj, same_lane);
	
	// debug
	cout << "maneuver cost of buffer closest_dist" << closest_dist << endl;
	
	return logistic(2 * kVehicleRadius / closest_dist);

}


double EgoCar::CalcManeuverCostOfSpeed(const Traffic & traffic, const FrenetTrajectory & traj) {
	
	double cost = 0.0;	
	double average_spd;
	double speed_sum = 0.0;
	
	for (int i = 0; i < traj.vel_vals.size(); i++) {
		speed_sum += traj.vel_vals[i];				
	}
	
	average_spd = speed_sum / traj.vel_vals.size();	
	
	if (average_spd < kSpeedLimit) {
		cost = (kSpeedLimit - average_spd) / kSpeedLimit;
	}		
		
	return cost;	

}

double EgoCar::CalcManeuverCostOfBraking(const Traffic & traffic, const FrenetTrajectory & traj) {
	
	double cost = 0.0;
	
	double average_braking = 0.0;
	
	for (int i = 0; i < traj.acc_vals.size(); i++) {
		if (traj.acc_vals[i] < 0)
			average_braking += traj.acc_vals[i];
		
	}
	
	average_braking /= traj.acc_vals.size();
		
	if (average_braking <= - kMaxAccel ) {
		cost = 1.0;
	}
	else if (average_braking < 0) {
		cost = average_braking / (-kMaxAccel);
	}
	
	return cost;
}

double EgoCar::CalcManeuverCostOfCollison(const Traffic & traffic, const FrenetTrajectory & traj) {

	double	nearest = CalcNearestDistToTraffic(traffic, traj);
    if (nearest < 2.0 * kVehicleRadius)
    	return 1.0;
    else
    	return 0.0;
}

double	EgoCar::CalcNearestDistToTraffic(const Traffic & traffic, const FrenetTrajectory & traj, bool same_lane){
	
	double closest_dist = 999.9;
	
	for (auto veh_iter = traffic.traffic_participants.begin(); veh_iter != traffic.traffic_participants.end(); veh_iter++) {
		double dist = CalcNearestDistToVehicle(veh_iter->second, traj, same_lane);
		if (dist < closest_dist) {
			
			closest_dist = dist;
		}
	}
	
	return closest_dist;

}



double EgoCar::CalcNearestDistToVehicle(const TrafficParticipant & vehicle, const FrenetTrajectory & traj, bool same_lane) {
	
	TrafficParticipant veh = vehicle;
	
	int traj_size = traj.time.size();
		
	double nearest_dist = 999.9;
	
	if (same_lane) {
		if (veh.lane_index == this->current_lane_id && 	veh.s < this->s) { // if the target vehicle is directly behind the ego vehicle, ignore it
			return ( this->s - veh.s ) ;
		}
	}
	
	for (int i = 0; i < traj_size; i++) {
		
		double time = traj.time[i];
		vector<double> frenet_state_veh  = veh.GetFrenetStateInTime(time);
		
		if (same_lane) {
			
			if (GetLaneIndex(traj.d_vals[i]     , kLaneWidth)     !=    GetLaneIndex(frenet_state_veh[1], kLaneWidth) ) {
			    continue;			    
			}				
		}
 		
		double dist = distance(frenet_state_veh[0], frenet_state_veh[1], 
							        traj.s_vals[i], traj.d_vals[i]);
		
		if (dist < nearest_dist) 
			nearest_dist = dist;		
	
	}
	
	return nearest_dist;

}







FrenetTrajectory EgoCar::GetFrenetTraj(FSMState state, const Traffic & traffic) {
	
	FrenetTrajectory traj;
	
	const double dT = 0.05; // sec
	const int    N  = 120 ; 
	double t  = dT; 
	const double kLateralVel = 2.0; // m/s
	
	TargetLane tgt_lane = GetTargetLane(state) ;
	int lane_shift = GetLaneShift(tgt_lane);
		
	int leading_veh_id = GetLeadVehId(traffic, tgt_lane);
	double target_spd = kSpeedLimit;
		
	auto iter = traffic.traffic_participants.find(leading_veh_id);
	if (iter != traffic.traffic_participants.end())
		target_spd = min(kSpeedLimit, (iter->second).vs);			
	
	double target_d   = double(this->current_lane_id + lane_shift) * kLaneWidth + kLaneWidth/2;
		
	double vel_last = this->spd;
	double s_last   = this->s;
	double d_last   = this->d;
	for (int i = 0; i < N ; i++, t += dT ) {	
	
		// time 
		traj.time.push_back(t);	
		
		// s_vals
		double rel_dist_last = 200.0;
		if (iter != traffic.traffic_participants.end()) {
			TrafficParticipant veh = iter->second;
			vector<double> tgt_veh_state_last = veh.GetFrenetStateInTime(t-dT);
			rel_dist_last = tgt_veh_state_last[0] - s_last;
		}			
		
		double vel;
		double acc;
		if (vel_last < 0.9 * target_spd && rel_dist_last >= 30) {
			acc = min(kMaxAccel, (target_spd - vel_last)/dT);			
		}
		else if (rel_dist_last < 15) {
			acc = -kMaxAccel;
		}
		else if (rel_dist_last < 30) {
			acc = -kMaxAccel/2;
		}
		else if (rel_dist_last < 40) {
			acc = max( -kMaxAccel/2, (target_spd - vel_last)/dT);
		}
		else {
			acc = 0.0;
		}
		
		vel = vel_last + acc * dT;
		
		double s = s_last + (vel_last + vel)/2 * dT;		
		traj.s_vals.push_back(s);
		traj.vel_vals.push_back(vel);
		traj.acc_vals.push_back(acc);
		
		s_last = s;
		vel_last = vel;
		
		// d_vals
		double d = d_last;
		if (d_last < target_d - 0.1)
			d = d_last + kLateralVel * dT;
		else if  (d_last > target_d + 0.1)
			d = d_last - kLateralVel * dT;
		
		traj.d_vals.push_back(d);
		d_last = d;		
		
	}
	
	return traj;	
}



void EgoCar::PlanManeuver(const Traffic & traffic) {

	
	this->current_lane_id = GetLaneIndex(this->d, kLaneWidth);	
				
	TransitManeuver(traffic); // get fsm_state
	
	
	int leading_veh_id = GetLeadVehId(traffic, GetTargetLane(this->fsm_state));
	//debug
	cout << endl
		 << "this->current_lane_id: " << this->current_lane_id << endl
		 << "this->fsm_state: " << this->fsm_state << endl;
	
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
  		this->maneuver.target_leading_veh_id = leading_veh_id;  		
  		this->maneuver.target_speed = INVALID_DOUBLE;
  		
  		auto iter = traffic.traffic_participants.find(this->maneuver.target_leading_veh_id);
		if (iter != traffic.traffic_participants.end())
			this->maneuver.target_speed = (iter->second).vs;
		else 
			this->maneuver.target_speed = kSpeedLimit;
  		          
        break;
  	  }

  	  case LCR: { 
  	  
  	    this->maneuver.target_lane_id = this->current_lane_id + 1;
  		this->maneuver.target_leading_veh_id = leading_veh_id;  		
  		this->maneuver.target_speed = INVALID_DOUBLE;
  		
  		auto iter = traffic.traffic_participants.find(this->maneuver.target_leading_veh_id);
		if (iter != traffic.traffic_participants.end())
			this->maneuver.target_speed = (iter->second).vs;
		else 
			this->maneuver.target_speed = kSpeedLimit;
  		
  		          
        break;
  	  }


  	  default: {
  	  	break;    
  	  }

	}
		
}


int  EgoCar::GetLeadVehId(const Traffic & traffic, const TargetLane tgt_lane) {
	
	int lead_veh_id = INVALID;
	
	double closest_distance = 10000; // large number
	
	int lane_shift = GetLaneShift(tgt_lane);
	
	for (auto mapIter = traffic.traffic_participants.begin(); mapIter != traffic.traffic_participants.end(); mapIter++) {
		
		// only interested in cars on target lane
		TrafficParticipant traffic_participant = mapIter->second;
		if (traffic_participant.lane_index != this->current_lane_id + lane_shift)
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



















/**************************************************************************************************************************/
/***                                       ***   Trajectory Planner   ***                                               ***/
/**************************************************************************************************************************/

void EgoCar::PrepRefPoints() {

	this->ref_ptsx.clear();
	this->ref_ptsy.clear();
	this->ref.clear();
		
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
		
		this->ref_vel = this->spd;
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
	
	int loop_times = 3;
	int kk = 1;
	if (this->maneuver.target_lane_id == current_lane_id) {
		loop_times = 1;
		kk = 1; // 2;
	}
	
	bool needs_emergency_brake = false;
	if (this->fsm_state == LK) {
		int leading_veh_id = GetLeadVehId(traffic, CURRENT);
		auto lead_iter = traffic.traffic_participants.find(leading_veh_id);
		if (lead_iter != traffic.traffic_participants.end()) {
			if (lead_iter->second.s - this->s < 15.0) {
				needs_emergency_brake = true;
			}		
		}
	}
		
	for (int i = 0; i < loop_times; i++) {
		
		target_s = this->s + double(i+3) * max(10.0, this->spd);
		if (loop_times == 1) {
			target_s = this->s + 30.0; 
		}
		
		if (needs_emergency_brake) {
			target_vs = max( 0.0, this->maneuver.target_speed - 5.0 );
			goals.push_back({target_s, target_d, target_vs});
		}
		else {
				
			for(int j = -5; j < 5; j ++) {
				double dv = 1.0; // m/s 
				target_vs = this->maneuver.target_speed + double(j) * dv; 
				if (target_vs < 0 || target_vs > .98 * kSpeedLimit)
					continue;					
					
				for (int k = -(kk-1); k < kk; k++) {
					double dd = 0.4; // m
					target_d = this->maneuver.target_lane_id * kLaneWidth + kLaneWidth/2 + double(k) * dd;
					goals.push_back({target_s, target_d, target_vs});
				}			
			}
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

		//double ref_vel; 

	
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
		

	    
		double target_x = 20; // before modify: 30
		double target_y = spln(target_x);
		double target_dist = sqrt(target_x * target_x + target_y * target_y);
	
	
		double x_add_on = 0;
		double y_add_on;
	
		double mod_vel = this->ref_vel;
	
		//cout << "this->spd" << this->spd << endl;
				
		//cout << "mod_vel" << mod_vel << endl;
		
		//double Max_dv = kMaxAccel * 0.02;
		double last_y = 0;
		
		for (int j = 0; j < kComparePathSize; j++) { 
	
			double pt_x;
			double pt_y;
			
			double accel;
			
			if (mod_vel < target_vs - kMaxAccel * 0.02 * 1.1) {
				mod_vel += kMaxAccel * 0.02;
				accel = kMaxAccel;
			}
			else if (mod_vel > target_vs + kMaxAccel * 0.02 * 1.1) {
				mod_vel -= kMaxAccel * 0.02;
				accel = - kMaxAccel;
			}
			else {
				mod_vel = target_vs;
				accel = 0;
			}
		
			//double N = target_dist / (0.02 * mod_vel); 
			
			double dx = target_x / target_dist * 0.02 * mod_vel ;
			double dy;
			x_add_on += dx;
			y_add_on  = spln(x_add_on);
			
			dy = y_add_on - last_y;
			last_y = y_add_on;
			double theta_car_frame = atan2(dy, dx);
			next_traj.acc_vals.push_back(accel/max(cos(theta_car_frame),0.1));
			
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

double EgoCar::CalcTrajCostOfAccel(const Traffic & traffic, const NextTrajectory & next_traj) {
	
	double mean_abs_acc = 0.0;
	for (int i = 0; i < next_traj.acc_vals.size(); i ++) {
		mean_abs_acc += abs(next_traj.acc_vals[i]);
	}
	
	mean_abs_acc /= next_traj.acc_vals.size();
	
	double cost = mean_abs_acc / kMaxAccel;
	
	//debug
	//cout << "CalcTrajCostOfAccel: " << cost << ", " ; cout << endl;
	
	return cost;
	
}


double EgoCar::CalcTrajCostOfBuffer(const Traffic & traffic, const NextTrajectory & next_traj, int steps ) {
	
	double closest_dist = CalcNearestDistToTraffic(traffic, next_traj, steps);
	
	return logistic(2 * kVehicleRadius / closest_dist);
}

double EgoCar::CalcNearestDistToTraffic(const Traffic & traffic, const NextTrajectory & next_traj, int steps ) {
	
	double closest_dist = 999.9;
	
	for (auto veh_iter = traffic.traffic_participants.begin(); veh_iter != traffic.traffic_participants.end(); veh_iter++) {
		double dist = CalcNearestDistToVehicle(veh_iter->second, next_traj, steps);
		if (dist < closest_dist) {
			
			closest_dist = dist;
		}
	}
	
	return closest_dist;
	
}



double EgoCar::CalcNearestDistToVehicle(const TrafficParticipant & vehicle, const NextTrajectory & next_traj, int steps ) {
	
	TrafficParticipant veh = vehicle;
	
	int traj_size = next_traj.x_vals.size() <= steps ? next_traj.x_vals.size() : steps ;
	
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

double EgoCar::CalcTrajCost(const Traffic & traffic, const NextTrajectory & next_traj) {
	
	double weight_buffer    = 10.0;
	double weight_buffer_short = 2.0;
	
	double weight_speed     = 50.0;
	double weight_collision = 200.0;
	double weight_lead_buffer = 100.0;
	
	double weight_goal_lane = 20.0; 
	double weight_accel     = 50.0; 
	
	double weight_sum    = weight_buffer       + 
						   weight_speed        + 
						   weight_collision    + 
						   weight_lead_buffer  +
						   weight_buffer_short + 
						   weight_goal_lane    +
						   weight_accel        ;
		
	//debug
	/*
	cout << "CalcTrajCostOfBuffer:     " << CalcTrajCostOfBuffer(traffic, next_traj)     << endl 
		 << "CalcTrajCostOfSpeed:      " << CalcTrajCostOfSpeed(traffic, next_traj)      << endl
		 << "CalcTrajCostOfCollison:   " << CalcTrajCostOfCollison(traffic, next_traj)   << endl
		 << "CalcTrajCostOfFollowDist: " << CalcTrajCostOfFollowDist(traffic, next_traj) << endl;
	*/
	
	return ( CalcTrajCostOfBuffer         (traffic, next_traj)     * weight_buffer       + 
			 CalcTrajCostOfSpeed          (traffic, next_traj)     * weight_speed        + 
			 CalcTrajCostOfCollison       (traffic, next_traj)     * weight_collision    + 
			 CalcTrajCostOfFollowDist     (traffic, next_traj)     * weight_lead_buffer  +
			 CalcTrajCostOfBuffer         (traffic, next_traj, 25) * weight_buffer_short +
			 CalcTrajCostOfDist2GoalLane  (traffic, next_traj)     * weight_goal_lane    +
			 CalcTrajCostOfAccel          (traffic, next_traj)     * weight_accel         ) / weight_sum;
			 


}

double EgoCar::CalcTrajCostOfDist2GoalLane(const Traffic & traffic, const NextTrajectory & next_traj) {	

	double target_d = this->maneuver.target_lane_id * kLaneWidth + kLaneWidth/2;
	
	double mse = 0;
	
	for (int i = 0; i < next_traj.d_vals.size(); i++) {
		double diff = target_d - next_traj.d_vals[i];
		mse += diff * diff;	

	}
	mse /= next_traj.d_vals.size();
	
	
	if (this->fsm_state == LK) {
		mse *= 16.0;
	}
	
	double cost = min(1.0, mse/(kLaneWidth*kLaneWidth));
	
	//debug
	//cout << "CalcTrajCostOfDist2GoalLane: " << cost << endl;
	return cost;

}


double EgoCar::CalcTrajCostOfFollowDist(const Traffic & traffic, const NextTrajectory & next_traj) {	
	
	double cost = 0.0;
			
	if (!isInvalid(this->maneuver.target_leading_veh_id) && this->fsm_state == LK) {	
			
			auto lead_iter = traffic.traffic_participants.find(this->maneuver.target_leading_veh_id);
			if (lead_iter != traffic.traffic_participants.end()) {
				double closest_dist = CalcNearestDistToVehicle(lead_iter->second, next_traj);	
				
				//debug 
				/*
				cout << "closest_dist within the next " << kComparePathSize * 0.02 << " sec: " << closest_dist << endl;
				for (int i = 0; i < next_traj.vel_vals.size(); i+=22) {
					cout << "next_traj.vel_vals[" << i << "]: " << next_traj.vel_vals[i] << endl;
					cout << "next_traj.s_vals[" << i << "]: " << next_traj.s_vals[i] << endl;
				}
				*/
				
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
	//debug
	//cout << "CalcTrajCostOfSpeed: " << mean_cost_of_speed << endl;
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
	/*
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
	*/
	
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
		 << "                   this->yaw:" << this->yaw << endl
		 << "                      ego s: " << this->s   << endl
		 << "--------------------------------------------------------------------------------" << endl;

          	
}
