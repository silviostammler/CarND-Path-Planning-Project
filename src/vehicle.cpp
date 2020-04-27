#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "cost.h"
#include "helpers.h"
#include "spline.h"

using std::string;
using std::vector;

using namespace std;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, double a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(vector<vector<Vehicle>> &predictions) {
  /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions vector. This is a vector of predicted
   *   vehicle trajectories. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and the final one
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   *
   */

  vector<string> states = successor_states();
  float cost;
  vector<float> costs;
  vector<vector<Vehicle>> final_trajectories;

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  return final_trajectories[best_idx];

}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
	cout << "Possible successor states!" << endl;
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
	
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, 
                                             vector<vector<Vehicle>> &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.

  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }

  return trajectory;
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  float next_pos = position_at(1);
  vector<Vehicle> trajectory = {Vehicle(this->lane,this->s,this->v,this->a,this->state), 
                                Vehicle(this->lane,next_pos,this->v,0,this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(vector<vector<Vehicle>> &predictions) {

	vector<Vehicle> trajectory;
	int new_lane = this->lane;
	double new_v;
	Vehicle rVehicle;
	bool too_close = get_vehicle_ahead(predictions,new_lane,rVehicle);
	if(too_close)
	{	
		new_v = ref_vel - 0.224;
	}
	else if(ref_vel < 49.5)
	{
		new_v = ref_vel + 0.224;
	}
	else if(ref_vel > 49.5)
	{
		new_v = ref_vel;
	}
	create_anchor_points(new_lane);
	trajectory = spline_interpolation(new_lane, new_v, "KL");
  
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, 
                                                     vector<vector<Vehicle>> &predictions) {
 
	vector<Vehicle> trajectory;

	int new_lane = this->lane;
	int intended_lane = this->lane + lane_direction[state];

	if((intended_lane>=0) && (intended_lane<=lanes_available-1))
	{
		double new_v;
		Vehicle rVehicle;

		bool not_free = get_vehicle_behind(predictions,intended_lane,rVehicle);
		// looking for free space
		if(!not_free)
		{	
			new_v = ref_vel;
		}
		else
		{
			new_v = ref_vel - 0.224;
		}
		
		create_anchor_points(new_lane);
		trajectory = spline_interpolation(new_lane, new_v, state);

	}
  
  return trajectory;

}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, 
                                                vector<vector<Vehicle>> &predictions) {
	
	vector<Vehicle> trajectory;
	int new_lane = this->lane + lane_direction[state];
	double new_v;
	Vehicle rVehicle;	

	// still free?
	bool not_free = get_vehicle_behind(predictions,new_lane,rVehicle);
	// looking for free space
	if(!not_free)
	{	
		new_v = ref_vel;
		create_anchor_points(new_lane);
		trajectory = spline_interpolation(new_lane, new_v, state);
	}
	// otherwise return empty trajectory
  return trajectory;

}

void Vehicle::increment(double dt = 0.02) {
  this->s = position_at(dt);
}

double Vehicle::position_at(double t) {
  return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(vector<vector<Vehicle>> &predictions, 
                                 int new_lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found behind the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (vector<vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = (*it).back();
    if ((temp_vehicle.lane == this->lane) && (temp_vehicle.s < this->s) 
        && ((this->s-temp_vehicle.s)<30) && (temp_vehicle.s > max_s)) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(vector<vector<Vehicle>> &predictions, 
                                int new_lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  double min_s = 100000;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (vector<vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = (*it).back();
    if ((temp_vehicle.lane == new_lane) && (temp_vehicle.s > this->s) 
        && ((temp_vehicle.s-this->s)<30) && (temp_vehicle.s < min_s)) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
  // Generates predictions for non-ego vehicles to be used in trajectory 
  //   generation for the ego vehicle.

	// commented out: predictions considering also intermediate timesteps
	/*
  vector<Vehicle> predictions;

	if(horizon==0)
	{
		predictions.push_back(*this);	
		return predictions;	
	}
	cout << endl;
	cout << "----------------------";
  for(int i = 0; i < horizon; ++i) {
		cout << endl;
		cout << "Prediction Vehicle i = " << i << endl;
    float next_s = position_at(i*0.02);
    float next_v = 0;
    if (i < horizon-1) {
      //next_v = position_at(i+1) - this->s;
			next_v = this->v;
    }
		printf("Non-ego car s = %.3f \t Non-ego car velocity = %.3f \n", next_s,next_v);
    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  }
  cout << "----------------------";
	cout << endl;
	*/

	// predictions only consider current timestep and last timestep of previous path
	vector<Vehicle> predictions;
	predictions.push_back(*this);	
	if(horizon==0)
	{
		return predictions;	
	}
	double next_s = position_at(horizon*0.02);
	double next_v = this->v;
	predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));

  return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory.back();
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
	// set reference velocity
	this->ref_vel = next_state.v;
}

void Vehicle::configure(vector<double> &road_data) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}

void Vehicle::update_localization_data(double c_x, double c_y, double c_s, double c_d, double c_yaw, double c_speed) {
	car_x = c_x;
	car_y = c_y;
	car_s = c_s;
	car_d = c_d;
	car_yaw = c_yaw;
	car_speed = c_speed;
}

void Vehicle::update_previous_path(vector<double> path_x, vector<double> path_y, double end_s, double end_d) {
	previous_path_x = path_x;
	previous_path_y = path_y;
	end_path_s = end_s;
	end_path_d = end_d;
	prev_size = previous_path_x.size();
	cout << "Prev Size = " << prev_size << endl;
	for(int i=0;i<prev_size;i++)
	{
		cout << "previous_path_x = " << path_x[i] << endl;
	}
}

void Vehicle::update_ref(double c_x, double c_y, double c_yaw) {
	ref_x = c_x;
	ref_y = c_y;
	ref_yaw = deg2rad(c_yaw);
}

void Vehicle::create_anchor_points(int new_lane) {

	// clear anchor points
	ptsx.clear();
	ptsy.clear();

	// if previous size is almost empty, use the car as starting reference
	if(prev_size < 2)
	{
		// Use two points that make the path tangent to the car
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);

	}
	//use the previous path's end point as starting reference
	else
	{
		// Redefine reference state as previous path end point
		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];

		double ref_x_prev = previous_path_x[prev_size-2];
		double ref_y_prev = previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

		// Use two points that make the path tangent to the previous path's end point
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);

	}

	// In Frenet add evenly spaced 30m spaced points ahead of the starting reference
	vector<double> next_wp0 = getXY(car_s+30,(2+4*new_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s+60,(2+4*new_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s+90,(2+4*new_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);	

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);	

	for(int i=0; i<ptsx.size(); ++i)
	{	
			//shift car reference angle to 0 degrees
			double shift_x = ptsx[i] - ref_x;
			double shift_y = ptsy[i] - ref_y;

			ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
			ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
	}

}


vector<Vehicle> Vehicle::spline_interpolation(int new_lane, double new_v, string new_state) {

	vector<Vehicle> trajectory;

	//create a spline
	tk::spline sp;

	// set (x,y) points to the spline
	sp.set_points(ptsx,ptsy);


	// Calculate how to break up spline points so that we travel at our desired reference velocity
	double target_x = 60.;
	double target_y = sp(target_x);
	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

	double x_add_on = 0;

	double x_point_hist = ref_x;
	double y_point_hist = ref_y;	
				
	// Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
	for(int i=1; i<=50-previous_path_x.size(); i++)
	{
		double N = (target_dist/(.02*new_v/2.24));
		double x_point = x_add_on+(target_x)/N;
		double y_point = sp(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// rotate back to normal after rotating it earlier
		x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
		y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		// calculate theta
		double theta = atan2(y_point-y_point_hist,x_point-x_point_hist);

		// acquire new Frenet coordinates
		vector<double> new_s_d= getFrenet(x_point, y_point, ref_yaw, map_waypoints_x, map_waypoints_y);
		//printf("x_point_spline = %.3f \t y_point_spline = %.3f \t s_spline = %.3f \t d_spline = %.3f\n", x_point, y_point, new_s_d[0],new_s_d[1]);
		//trajectory.push_back(Vehicle(new_lane, new_s_d[0], new_v, 0., new_state));	

		Vehicle rVehicle = Vehicle(new_lane, new_s_d[0], new_v, 0., new_state);
		// use cartesian coordinates directly in trajectory generation
		rVehicle.car_x = x_point;
		rVehicle.car_y = y_point;
		trajectory.push_back(rVehicle);

		x_point_hist = x_point;
		y_point_hist = y_point;
	}					

	return trajectory;
}

bool Vehicle::check_acc(double v_init, double v_final, int timesteps) {

	double acc = (v_final - v_init)/(2.24*timesteps*0.02);
	cout << "Acceleration = " << acc << endl;

	if(acc>max_acceleration)
	{
		cout << "Acceleration limit exceeded!" << endl;
		return false;
	}
	else
	{
		return true;
	}

}
