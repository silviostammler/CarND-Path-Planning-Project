#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(Vehicle &vehicle, 
                     vector<vector<Vehicle>> &predictions, 
                     const vector<Vehicle> &trajectory);

float goal_distance_cost(Vehicle &vehicle,  
                         const vector<Vehicle> &trajectory,  
                         vector<vector<Vehicle>> &predictions, 
                         map<string, float> &data);

float inefficiency_cost(Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        vector<vector<Vehicle>> &predictions, 
                        map<string, float> &data);

float collision_cost(Vehicle &vehicle, 
                     const vector<Vehicle> &trajectory, 
                     vector<vector<Vehicle>> &predictions, 
                     map<string, float> &data);

float lane_speed(Vehicle &vehicle,vector<vector<Vehicle>> &predictions, int lane);

map<string, float> get_helper_data(Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   vector<vector<Vehicle>> &predictions);

#endif  // COST_H
