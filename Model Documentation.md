# CarND-Path-Planning-Project Writeup

Self-Driving Car Engineer Nanodegree Program

## Intro
This writeup provides a brief description of how the project is done. Basically, the solution follows the traditional robotics approach taught in Udacity's program and consists of three key parts (pipeline):
- Traffic prediction
- Maneuver planning
- Trajectory planning

## Source Files Brief
#### `helper.h` `helper.cpp` `spline.h`
Common auxiliary functions are placed here. Some functions in the original `main.cpp` file such as `NextWaypoint()` are also moved here to make code in the `main()` cleaner.
`spline.h` is a header file containing open source code for spline fitting, downloaded [here](http://kluge.in-chemnitz.de/opensource/spline/).
 #### `traffic_prediction.h` `traffic_prediction.cpp`
Here defined and implemented are two classes: `TrafficParticipant` (individual surrounding car) and `Traffic` (collection of surrounding cars, organized using standard C++ data structure map). Traffic prediction is done here (`GetFrenetStateInTime()`).   
#### `ego_car.h` `ego_car.cpp`
Defined and implemented here are mainly the `EgoCar` class. All maneuver planning and trajectory planning related functions, including corresponding cost functions, are implemented as member functions of this class.
#### `main.cpp`
The main code is added in the TODO section.

Please note that `CMakeLists.txt` is changed due to the way the code is organized as described above.

## Pipeline Brief  
#### 1. Traffic Prediction
Traffic prediction is supposed to consists of two parts: Maneuver detection (`TrafficParticipant::DetectManeuver()`) + Trajectory prediction (`TrafficParticipant::GetFrenetStateInTime`), both realized as member functions of class `TrafficParticipant`. For simplicity and in light of the characteristics of the simulator, the maneuver detection is basically omitted (API is kept for potential future implementation) with all traffic participant "detected" as keeping its current lane.

#### 2. Maneuver Planning
The whole path planning is realized with `EgoCar::PlanPath()`, which in turn consists of `EgoCar::PlanManeuver()` (maneuver planning) and `EgoCar::GenerateTrajectory()` (trajectory planning).

For *maneuver planning*, three maneuvers were considered, namely, Lane Keeping (LK), Lane Change to the Left (LCL), and Lane Change to the Right (LCR), which means there is no preparation stage for lane changes, the ego car will only change lanes when certain conditions are meet (corresponding cost function is the lowest).

The main steps of maneuver planning include:
- Get the next possible maneuver/state `EgoCar::GetSuccessorStates()`
- For each possible maneuver/state, generate a possible trajectory in Frenet coordinate `EgoCar::GetFrenetTraj()`
- For each possible maneuver/state, calculate the corresponding cost `EgoCar::CalcManeuverCost()`, which is basically a weighted sum of a bunch of cost functions.

#### 3. Trajectory Planning
For *trajectory planning*, the main steps include:
- Generate a bunch of possible goals with `EgoCar::GeneratePossibleGoals()`, with each goal consisting of a target s, a target d, and target speed in s direction `{target_s, target_d, target_vs}`. ...
The variation is mainly in the target_vs dimension: for the planned maneuver, there will be a default `target_vs` (generated by maneuver planner) that's typically the min of the speed limit and the leading vehicle speed of target lane. When trajectory planner receives this `target_vs`, it then generate a bunch of goals around this default `target_vs`.
- Generate a bunch of possible trajectories according to the target goals from the last step, with `EgoCar::GeneratePossibleTrajectories()`. Basically, the trajectories are generated using the spline method taught in the project walkthrough video. `target_s` and `target_d` are used to generate a spline (code line 764 - 785 in ego_car.cpp), then points are picked from this spline by gradually increasing the distance while limiting the speed of ego car(code line 811 - 861 in ego_car.cpp).The higher the `target_vs` is, the sparser the trajectory points will be.
- Find the best trajectory using `EgoCar::FindBestTrajectory()` with the lowest cost function calculated by `EgoCar::CalcTrajCost()`.

## How I addressed points in Project Rubric
#### "_Max Acceleration and Jerk are not Exceeded._"
As explained in the second point of the _Trajectory Planning_ section, when generating every possible trajectory, points are picked from the generated spline by gradually increasing the distance considering the limitation of the acceleration of the ego car. So for every trajectory, acceleration and jerk is supposed to be controlled.
#### "_The car drives according to the speed limit._"
Every generated trajectory corresponds to a target ego speed `target_vs`. By generating multiple candidate trajectories, each corresponding to a different `target_vs`, and then penalizing each trajectory by cost function `EgoCar::CalcTrajCostOfSpeed()`, the trajectory with the ideal speed (slightly less than the speed limit, in my case, 45 mph was chosen) will be favored.
#### "_The car stays in its lane, except for the time between changing lanes._"
This can be easily done by choosing target d `target_d` to be around the target lane center when generating the trajectories. (initially I chose a couple of different `target_d` around the target lane center, but found it to increase the computational complexity too much since 3 `target_d` vs 1 `target_d` means triple possible trajectories generated, and no apparent benefit seemed to have been brought so eventually decided to keep only 1 `target_d`, i.e., the target lane center).
When changing lanes, multiple trajectories corresponding to different target s `target_s` will be generated, and cost function `EgoCar::CalcTrajCostOfDist2GoalLane()` will generally penalize those that have a larger mean distance to the target lane center, thus favoring the one with shorter converging time, making sure the car won't stay outside the lane for too long.
#### "_The car is able to change lanes_"
One of the motives of a car changing lanes is that there is a slow vehicle ahead and it can't go as fast as it could. In maneuver planner, each possible trajectory will be evaluated by cost function `EgoCar::CalcManeuverCostOfSpeed()` and the one with painfully slow speed will be heavily penalized. So in the slow vehicle case, maneuver/state LK will have a larger cost of speed compared to LCL/LCR if the vehicles on the adjacent left/right lane ahead are faster, or better, no vehicles on the adjacent lane(s). Cost function `EgoCar::CalcManeuverCostOfBraking()` has a similar purpose.
#### "_Car does not have collisions._"
To make sure the ego car only changes lanes when possible and would not collide with other vehicles, cost functions `EgoCar::CalcManeuverCostOfBuffer()` and `EgoCar::CalcManeuverCostOfCollison()` are implemented to penalize those lane change maneuvers that are likely to incur a collision.
When doing the Lane Keeping, cost function `EgoCar::CalcTrajCostOfFollowDist()` `EgoCar::CalcTrajCostOfBuffer()` are implemented to ensure that the ego vehicle is following the front vehicle with an ideal distance and not too close and plan trajectory that's not close to other vehicles to prevent collision. 
