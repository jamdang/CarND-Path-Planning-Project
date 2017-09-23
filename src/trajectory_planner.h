#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "helper_funcs.h"
#include "traffic_prediction.h"
#include "maneuver_planner.h"
#include "spline.h"










void GenerateTrajectory( EgoCar & ego_car, const Traffic             & traffic        , const Map & map, 
										   const std::vector<double> & previous_path_x, const std::vector<double> & previous_path_y, 										   
											     std::vector<double> & next_x_vals    ,       std::vector<double> & next_y_vals);











#endif
