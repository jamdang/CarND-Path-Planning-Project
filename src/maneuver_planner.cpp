#include "maneuver_planner.h"


void PlanManeuver(EgoCar & ego_car, const Traffic & traffic) {

	
	
	ego_car.current_lane_id = GetLaneIndex(ego_car.d, kLaneWidth);	
	
	int leading_veh_id = GetLeadVehId(ego_car, traffic);
	
	FSMState fsm_state;
	
	// temp code
	
	fsm_state = LK;
	
	switch (fsm_state) {

  	  case LK: { 
  	  
  	    ego_car.maneuver.target_lane_id = ego_car.current_lane_id;
  		ego_car.maneuver.target_leading_veh_id = leading_veh_id;
  		ego_car.maneuver.target_speed = kSpeedLimit;
  		ego_car.maneuver.time_to_reach_target = 5.0;
  		
  		if (leading_veh_id == INVALID) {
  			ego_car.maneuver.target_speed = INVALID_DOUBLE;
  			ego_car.maneuver.time_to_reach_target = INVALID_DOUBLE; 
  		}
  		          
        break;
  	  }


  	  default: {
    
  	  }

	}
		
}



int  GetLeadVehId(const EgoCar & ego_car, const Traffic & traffic) {
	
	int lead_veh_id = INVALID;
	
	double closest_distance = 10000; // large number
	
	for (auto mapIter = traffic.traffic_participants.begin(); mapIter != traffic.traffic_participants.end(); mapIter++) {
		
		// only interested in cars on ego lane
		TrafficParticipant traffic_participant = mapIter->second;
		if (traffic_participant.lane_index != ego_car.current_lane_id)
			continue; 
		// only interested in cars in the front
		double dist = traffic_participant.s - ego_car.s;
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