#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, double s, double v, double a, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  vector<Vehicle> choose_next_state(vector<vector<Vehicle>> &predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, 
                                      vector<vector<Vehicle>> &predictions);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(vector<vector<Vehicle>> &predictions);

  vector<Vehicle> lane_change_trajectory(string state, 
                                         vector<vector<Vehicle>> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, 
                                              vector<vector<Vehicle>> &predictions);

  void increment(double dt);

  double position_at(double t);

  bool get_vehicle_behind(vector<vector<Vehicle>> &predictions, int new_lane, Vehicle &rVehicle);

  bool get_vehicle_ahead(vector<vector<Vehicle>> &predictions, int new_lane, Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> &trajectory);

  void configure(vector<double> &road_data);

	void update_localization_data(double c_x, double c_y, double c_s, double c_d, double c_yaw, double c_speed);

	void update_previous_path(vector<double> path_x, vector<double> path_y, double end_s, double end_d);

	void update_ref(double c_x, double c_y, double c_yaw);

	void create_anchor_points(int new_lane);

	vector<Vehicle> spline_interpolation(int new_lane, double new_v, string new_state);

	bool check_acc(double v_init, double v_final, int timesteps);

  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, 
                                     {"LCR", 1}, {"PLCR", 1}};

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane, goal_lane, lanes_available;

	double goal_s;

  double s, v, target_speed, a, max_acceleration;

  string state;

	int prev_size;
	
	// localization data
	double car_x, car_y, car_s, car_d, car_yaw, car_speed;
	
	// previous path data
	vector<double> previous_path_x;
	vector<double> previous_path_y;
	double end_path_s;
	double end_path_d;

	// anchor points
	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x, ref_y, ref_yaw;
	double ref_vel;

	vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

};

#endif  // VEHICLE_H
