#include <uWS/uWS.h>
#include <fstream>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.h"


using namespace std;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
 

int main() {

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

	// impacts default behavior for most states
	double SPEED_LIMIT = 50.0;

	// At each timestep, ego can set acceleration to value between 
	//   -MAX_ACCEL and MAX_ACCEL (m/s^2)
	double MAX_ACCEL = 10.0;

	// s coordinate of goal and goal lane
	double goal_s = max_s;
  int goal_lane = 1;

	// initial s
	double start_s = 124.834;

	// start in lane 1
	int lane = 1;

	// Have a reference velocity to target
	double ref_vel = 0.0; //mph

	// configuration data: speed limit, num_lanes, goal_s, goal_lane, 
  //   and max_acceleration
  int num_lanes = 3;
  vector<double> ego_config = {SPEED_LIMIT,num_lanes,goal_s,goal_lane,MAX_ACCEL};

	// construct instance of ego car
	Vehicle ego = Vehicle(lane, start_s, ref_vel, 0);
	ego.configure(ego_config);
  ego.state = "KL";
	ego.ref_vel = ref_vel;

	ego.map_waypoints_x = map_waypoints_x;
  ego.map_waypoints_y = map_waypoints_y;
  ego.map_waypoints_s = map_waypoints_s;
  ego.map_waypoints_dx = map_waypoints_dx;
  ego.map_waypoints_dy = map_waypoints_dy;

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ego]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
					
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
					int prev_size = previous_path_x.size();
					

					if(prev_size > 0)
					{
						car_s = end_path_s;
					}

					// update ego car's localization data and previous path
					ego.update_localization_data(car_x,car_y,car_s,car_d,car_yaw,car_speed);
					ego.update_previous_path(previous_path_x,previous_path_y,end_path_s,end_path_d);
					ego.s = car_s;

					bool  too_close = false;

				
					vector<vector<Vehicle>> predictions;

					for(int i=0; i<sensor_fusion.size(); i++)
					{
						// do the prediction for each non-ego car

						double d = sensor_fusion[i][6];
						int lane_non_ego = d/4;
			
						double s = sensor_fusion[i][5];						

						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt(vx*vx+vy*vy);

						// construct instance for non-ego car
						Vehicle non_ego = Vehicle(lane_non_ego, s, check_speed, 0);

						predictions.push_back(non_ego.generate_predictions(prev_size));

			
					}
					
					ego.update_ref(car_x,car_y,car_yaw);
					
					

					// behavior planning and trajectory generation
					vector<Vehicle> trajectory = ego.choose_next_state(predictions);
					ego.realize_next_state(trajectory);


          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

					// Start with all of the previous path points from last time
					for(int i=0; i<previous_path_x.size(); i++)
					{	
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);

					}

				
					// Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
					for(int i=1; i<=50-previous_path_x.size(); i++)
					{	
						// convert Frenet coordinates of trajectory points back to XY coordinates
						//vector<double> xy_point_final = getXY(trajectory[i-1].s,(2+4*trajectory[i-1].lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
						//next_x_vals.push_back(xy_point_final[0]);
						//next_y_vals.push_back(xy_point_final[1]);

						// use computed Cartesian coordinates of function spline_interpolation (getFrenet and getXY don't seem to be exact inverses to each other)
						next_x_vals.push_back(trajectory[i-1].car_x);
						next_y_vals.push_back(trajectory[i-1].car_y);
						
					}	

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
