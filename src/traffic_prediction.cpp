#include "traffic_prediction.h"
using namespace std;
#include <iostream>

/****************************/
/* class TrafficParticipant */
/****************************/

/* constructor */
TrafficParticipant::TrafficParticipant(vector<double> & fused_vehicle, Map & map) {

	id = int(fused_vehicle[0]);
	x  = fused_vehicle[1];
	y  = fused_vehicle[2];
	vx = fused_vehicle[3];
	vy = fused_vehicle[4];
	s  = fused_vehicle[5];
	d  = fused_vehicle[6];
	lane_index = GetLaneIndex(d, kLaneWidth);//int((d-fmod(d,kLaneWidth))/kLaneWidth);
	rel_d = fmod(d,kLaneWidth)/kLaneWidth;
	
	vector<double> frenet_speed = getFrenetSpeed(x, y, vx, vy, map);
	
	vs = frenet_speed[0];
	vd = frenet_speed[1];
	
	target_lane = CURRENT;
	
	// debug
	/*
	cout << "id: " << id << ", d:" << d << ", vd: " << vd << endl 
		 << "s: " << s << ", vs: " << vs << endl 
		 << "v: " << sqrt(vx*vx + vy*vy) << endl
		 << "lane index: " << lane_index << endl
		 << "rel_d: " << rel_d << endl; 
	*/
	
}

/* destructor */
TrafficParticipant::~TrafficParticipant() {
	
}

void TrafficParticipant::DetectManeuver() {
	
	// temporary code
	target_lane = CURRENT;
	
}

vector<double> TrafficParticipant::GetFrenetStateInTime(double time) {
	
	double frenet_s = 0.0;
	double frenet_d = 0.0;
	
	frenet_s = s + vs * time;
	frenet_d = d + vd * time;
	
	// modify d according to the traffic vehicle's detected/estimated maneuver
	
	switch (target_lane) {

  	  case LEFT: { 
  		
  		if (frenet_d < (lane_index - 1 ) * kLaneWidth + kLaneWidth/2)
  			frenet_d = (lane_index - 1 ) * kLaneWidth + kLaneWidth/2;
          
        break;
  	  }

  	  case RIGHT: {
  	  
  	    if (frenet_d > (lane_index + 1 ) * kLaneWidth + kLaneWidth/2)
  			frenet_d = (lane_index + 1 ) * kLaneWidth + kLaneWidth/2;
          
        break;  		
  	  }
  	  
  	  case CURRENT: { 
  		
  		if (frenet_d < lane_index * kLaneWidth + 0.01)
  			frenet_d = lane_index * kLaneWidth + 0.01;
  			
  		if (frenet_d > (lane_index + 1) * kLaneWidth - 0.01)
  			frenet_d = (lane_index + 1) * kLaneWidth - 0.01;
          
        break;
  	  }

  	  default: {
    
  	  }

	}	

	return {frenet_s, frenet_d};

}

/***************************/
/*      class Traffic      */
/***************************/

/* constructor */
Traffic::Traffic(vector< vector<double> > & sensor_fusion, Map & map) { 
// sensor_fusion: vector<vector<double>>, for each vehicle: [ id, x, y, vx, vy, s, d]
	
		
	//traffic_participants.clear();
	//self->map = map;
	
	for (int i = 0; i < sensor_fusion.size(); i++) {
	
		double d = sensor_fusion[i][6];
		
		if (d < -1e-5) // noticed some reported (stationary) fused object has a large negative d value, so I filter it 
			continue;
	
		TrafficParticipant traffic_participant(sensor_fusion[i], map);
		
		traffic_participant.DetectManeuver();
		
		//traffic_participants[traffic_participant.id] = traffic_participant;
		traffic_participants.insert( make_pair(traffic_participant.id, traffic_participant) );
	}
	
	
	
	
}

/* destructor */
Traffic::~Traffic() {

}