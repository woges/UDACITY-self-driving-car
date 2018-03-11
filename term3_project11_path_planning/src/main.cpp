#include "vehicle.h"
#include "cost_functions.h"
#include "constants.h"
#include "helper.h"
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "coordinates.h"

using namespace std;

// for convenience
using json = nlohmann::json;

static const string stSTART_STATE = "KL";
static const double dSTART_ACC = 0.;
static const double dSTART_ACC_LAST = -99.;

int gl_iSensor_fusion_size;
int iNext_Lane = -1;
vector<vector<double>> gl_vSensor_fusion;
vector<vector<vector<double>>> gl_vLane_traffic(3);
vector<vector<vector<double>>> gl_vLane_traffic_last(3);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;



  Vehicle ego_car;
  ego_car.state = stSTART_STATE;
  ego_car.a_last = dSTART_ACC_LAST;
  ego_car.a = dSTART_ACC;

  //##########################################################

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  //double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  h.onMessage([&ego_car, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    bool bDebug_main = false;

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
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          	//auto sensor_fusion = j[1]["sensor_fusion"];
            int prev_size = previous_path_x.size();

            ego_car.iNum_Steps = 50 - prev_size;
            ego_car.iNum_rest_steps = prev_size;
            // set all informations about the car to the ego car
          	ego_car.vehicle_set(car_x, car_y, car_s, car_d, car_yaw, car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d);
            // set all informations about all other cars to a vector
            gl_vSensor_fusion = sensor_fusion;
            gl_iSensor_fusion_size = sensor_fusion.size();
            gl_vLane_traffic_last = gl_vLane_traffic;
            // determine for each lane the car ahead and behind
            Lane_Traffic(gl_vSensor_fusion, gl_iSensor_fusion_size, gl_vLane_traffic, ego_car, gl_vLane_traffic_last);

            //cout for debug
            if(bDebug_main){
              cout << "prev_size: " << prev_size << endl;
              cout << "ego car lane : " << ego_car.lane << "\t s position: \t" << ego_car.s << "\t v : \t" << ego_car.v << "\t timestep calculated: \t" << double(ego_car.iNum_Steps * ego_car.dTimestep) << "\t a calculated: \t" << ego_car.a << endl;
              cout << "ego lane last: " << ego_car.lane_last << "\t s position: \t" << ego_car.s_last << "\t v : \t" << ego_car.v_last << "\t timestep calculated: \t" << double(ego_car.iNum_Steps * ego_car.dTimestep) << "\t a calculated: \t" << ego_car.a_last << endl;

              cout << "gl_vLane_traffic !!!" << endl;
              for (int k = 0; k < 3; k++){
                cout << "lane: " << k << endl;
                for(int p = 0; p < gl_vLane_traffic[k].size(); p++){
                 vector<double> car_test = gl_vLane_traffic[k][p];
                 cout << "car_set id: " << car_test[0] << "\t lane:" << car_test[9] <<"\t s: " <<car_test[5]  << "\t s_dist: " << car_test[8]<< "\t v: " << car_test[7]<< "\t acc: " << car_test[11]<<endl;
                }
              }

              cout << "gl_vLane_traffic_last !!!" << endl;
              for (int k = 0; k < 3; k++){

                cout << "lane: " << k << endl;
                for(int p = 0; p < gl_vLane_traffic_last[k].size(); p++){
                 vector<double> car_test = gl_vLane_traffic_last[k][p];
                 cout << "car_set id: " << car_test[0] << "\t lane:" << car_test[9] <<"\t s: " <<car_test[5]  << "\t s_dist: " << car_test[8]<< "\t v: " << car_test[7]<< "\t acc: " << car_test[11]<<endl;
                }
              }
            }

//############### Behavioral Planner
            // determine the best choice for driving
            BehavioralPlanner(gl_vLane_traffic, ego_car, iNext_Lane);

//############### Path Planner
            // calculate the best path, according to all restrictions (acc, speed ..), for this choice
            vector<vector<double>> next_vals;
            next_vals = PathPlanner(gl_vLane_traffic, ego_car, iNext_Lane, map_waypoints_x, map_waypoints_y, map_waypoints_s);

//###################################################

          	json msgJson;
          	msgJson["next_x"] = next_vals[0];
          	msgJson["next_y"] = next_vals[1];

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
