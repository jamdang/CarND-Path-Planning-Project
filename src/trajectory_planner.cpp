#include "trajectory_planner.h"
using namespace std;
#include <iostream>





void GenerateTrajectory( EgoCar & ego_car, const Traffic        & traffic        , const Map            & map, 
						    		 	   const vector<double> & previous_path_x, const vector<double> & previous_path_y, 										 
									    		 vector<double> & next_x_vals    ,       vector<double> & next_y_vals)
{
	
	// initialize reference points ---
    double ref_x = ego_car.x;
    double ref_y = ego_car.y;
	double ref_yaw = deg2rad(ego_car.yaw);
          	
	// push reference points and its previous point to a list --- 
	vector<double> ptsx;
	vector<double> ptsy;
	
	int prev_path_size = previous_path_x.size();
	
	// debug
	//cout << "previous path size: " << prev_path_size << endl;
	
	int MaxPrevPathSize = 47;
	int PathSize        = 50;
	
	prev_path_size = min(prev_path_size, MaxPrevPathSize);
	
	if (prev_path_size < 2) {
		double prev_car_x = ego_car.x - 1.0 * cos(ego_car.yaw);
		double prev_car_y = ego_car.y - 1.0 * sin(ego_car.yaw);
		
		ptsx.push_back(prev_car_x);
		ptsx.push_back(ego_car.x);
		
		ptsy.push_back(prev_car_y);
		ptsy.push_back(ego_car.y);
	}
	
	else {
		// redefine reference points
		
		ref_x = previous_path_x[prev_path_size - 1];
		ref_y = previous_path_y[prev_path_size - 1];
		
		double prev_ref_x = previous_path_x[prev_path_size - 2];
		double prev_ref_y = previous_path_y[prev_path_size - 2];
		
		ref_yaw = atan2( (ref_y - prev_ref_y) , (ref_x - prev_ref_x) );
		
		ptsx.push_back(prev_ref_x);
		ptsx.push_back(ref_x);
		
		ptsy.push_back(prev_ref_y);
		ptsy.push_back(ref_y);
		
	}
				
	// push three more points to the list , each spaced 30 m --- 
	int ind_lane = ego_car.maneuver.target_lane_id;
	double taget_d = kLaneWidth * ind_lane + kLaneWidth/2;
	
	double s_pt1;
	double s_pt2;
	double s_pt3;
	
	double ref_vel; 
	
	if (isInvalid(ego_car.maneuver.target_leading_veh_id)) {		
		ref_vel = 49 * MPH_to_MPS; // convert miles/h to m/s	
	}
	else {	
		auto iter = traffic.traffic_participants.find(ego_car.maneuver.target_leading_veh_id);
		if (iter != traffic.traffic_participants.end())
			ref_vel = (iter->second).vs;		
	}
	
	if (isInvalid(ego_car.maneuver.time_to_reach_target)) {
		s_pt1 = ego_car.s + 30;
	}
	else {
		s_pt1 = ego_car.s + 30; //ego_car.maneuver.time_to_reach_target * ref_vel;
	}
	
	s_pt2 = s_pt1 + 30;
	s_pt3 = s_pt2 + 30;
	
	//debug
	cout << "s_pt1: " << s_pt1 << endl
		 << "ref_vel: " << ref_vel << endl
		 << "ego_car.maneuver.target_leading_veh_id: " << ego_car.maneuver.target_leading_veh_id << endl
		 << "ego_car.maneuver.time_to_reach_target: " << ego_car.maneuver.time_to_reach_target << endl
		 << "isInvalid(ego_car.maneuver.target_leading_veh_id): " << isInvalid(ego_car.maneuver.target_leading_veh_id) << endl;
	
	vector<double> next_pt1 = getXY(s_pt1, taget_d, *(map.waypoints_s), *(map.waypoints_x), *(map.waypoints_y));
	vector<double> next_pt2 = getXY(s_pt2, taget_d, *(map.waypoints_s), *(map.waypoints_x), *(map.waypoints_y));
	vector<double> next_pt3 = getXY(s_pt3, taget_d, *(map.waypoints_s), *(map.waypoints_x), *(map.waypoints_y));
	
	ptsx.push_back(next_pt1[0]);
	ptsx.push_back(next_pt2[0]);
	ptsx.push_back(next_pt3[0]);
	
	ptsy.push_back(next_pt1[1]);
	ptsy.push_back(next_pt2[1]);
	ptsy.push_back(next_pt3[1]);
	
	// transform ptsx, ptsy to car frame
	
	vector<double> ptsx_car_frame;
	vector<double> ptsy_car_frame;
	
	vector<double> ref = {ref_x, ref_y, ref_yaw};
										
	TransformGlobalToCarFrame(ptsx, ptsy, ptsx_car_frame, ptsy_car_frame, ref);
				
	// create a spline ---
	tk::spline spln;
	spln.set_points(ptsx_car_frame, ptsy_car_frame);
				
	// push the previous (uncovered) path to the next path vector ---
	for (int i = 0; i < prev_path_size; i++) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}
				
	// 
	
	double target_x = 30;
	double target_y = spln(target_x);
	double target_dist = sqrt(target_x * target_x + target_y * target_y);
	
	
	
	
	double x_add_on = 0;
	double y_add_on;
	
	double mod_vel = ego_car.ref_vel;
	
	cout << "ego_car.spd" << ego_car.spd << endl;
	
	if (mod_vel < ref_vel - 0.225*MPH_to_MPS) {
		mod_vel += 0.222*MPH_to_MPS;
	}
	else if (mod_vel > ref_vel + 0.225*MPH_to_MPS) {
		mod_vel -= 0.222*MPH_to_MPS;
	}
	else {
		mod_vel = ref_vel;
	}
		
	cout << "mod_vel" << mod_vel << endl;
	
	for (int i = 1; i < PathSize - prev_path_size; i++) { 
	
		double pt_x;
		double pt_y;
		

		
		double N = target_dist / (0.02 * mod_vel);
		
		x_add_on += target_x / N;
		y_add_on  = spln(x_add_on);

		TransformCarFrameToGlobal(x_add_on, y_add_on, pt_x, pt_y, ref);
		
		next_x_vals.push_back(pt_x);
		next_y_vals.push_back(pt_y);
	
	}
    
    ego_car.ref_vel = mod_vel;
          	
}
