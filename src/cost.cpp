#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "vehicle.h"

using std::string;
using std::vector;

using namespace std;

/**
 * TODO: change weights for cost functions.
 */
const float COMFORT = pow(10, 2);
const float EFFICIENCY = pow(10, 3);
const float SAFETY = pow(10, 20);

// Here we have provided three possible suggestions for cost functions, but feel 
//   free to use your own! The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in 
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.

float change_lanes_cost(Vehicle &vehicle, 
                         const vector<Vehicle> &trajectory, 
                         vector<vector<Vehicle>> &predictions, 
                         map<string, float> &data) {
  // Cost of being changing lanes becomes larger
	// The intention is to keep the current lane and to change lanes only if necessary

  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0) {
		cost = 1 - 2*exp(-(abs(2.0*vehicle.lane - data["intended_lane"] 
         - data["final_lane"])));
  } else {
    cost = 1;
  }

  return cost;
}

float inefficiency_cost(Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        vector<vector<Vehicle>> &predictions, 
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the limiting speed for a lane. 

  float proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed;
  }

  float proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }
    
  float cost = (2.0*vehicle.target_speed - proposed_speed_intended 
             - proposed_speed_final)/vehicle.target_speed;

  return cost;
}


float collision_cost(Vehicle &vehicle, 
                     const vector<Vehicle> &trajectory, 
                     vector<vector<Vehicle>> &predictions, 
                     map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane. 
  // This function is very similar to what you have already implemented in 
  //   the "Implement a Second Cost Function in C++" quiz.
	cout << "Collision cost" << endl;
	
	Vehicle rVehicle1;
	Vehicle rVehicle2;
	bool too_close = vehicle.get_vehicle_ahead(predictions, data["final_lane"], rVehicle1) && vehicle.get_vehicle_behind(predictions, data["final_lane"], rVehicle2);
	
	if(!too_close)
	{
		return 0.;
	}

	// distance between ego car and nearest vehicle in front
	double dist_ahead = rVehicle1.s - vehicle.s;

	// distance between ego car and nearest vehicle behind
	double dist_behind = vehicle.s - rVehicle2.s;

	float cost = (-1./30.)*dist_ahead - (1./30.)*dist_behind + 2.;

  return cost;
}

float lane_speed(Vehicle &vehicle, vector<vector<Vehicle>> &predictions, int lane) {
  // Determining the limiting speed for a given lane by simply detecting the nearest car ahead and taking the its velocity.
	Vehicle rVehicle;
	bool ahead = vehicle.get_vehicle_ahead(predictions, lane, rVehicle);
	if(ahead)
	{
		return rVehicle.v;
	}
	else
	{
		return -1.0;
	}

}

float calculate_cost(Vehicle &vehicle, 
                     vector<vector<Vehicle>> &predictions, 
                     const vector<Vehicle> &trajectory) {

  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, 
                                                       predictions);
  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(Vehicle &, const vector<Vehicle> &, 
                             vector<vector<Vehicle>> &, 
                             map<string, float> &)
    >> cf_list = {change_lanes_cost, inefficiency_cost, collision_cost};
  vector<float> weight_list = {COMFORT, EFFICIENCY, SAFETY};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, 
                                               trajectory_data);
    cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   vector<vector<Vehicle>> &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or 
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help 
  //   differentiate between planning and executing a lane change in the 
  //   cost functions.
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory.back();
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
    
  return trajectory_data;
}
