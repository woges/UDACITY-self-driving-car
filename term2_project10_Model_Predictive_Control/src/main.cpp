#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <stdlib.h>

// for convenience
using json = nlohmann::json;
using namespace std ;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial
double polyeval(Eigen::VectorXd coeffs, double x, bool bSlope) {
  double result = 0.0;
    if(!bSlope){
      for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
      }
    } else {
      for (int i = 1; i < coeffs.size(); i++) {
        result += i*coeffs[i] * pow(x, i-1);
      }
    }
  return result;
}
// Evaluate the 1st slope of a polynomial
/*
double polyslope_eval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += i*coeffs[i] * pow(x, i-1);
  }
  return result;
}
*/
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  /***
  A. Initialize MPC Class
  ***/
  mpc.Init();

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

  /***
  B. Data Input from simulator
  ***/
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v = v*0.44704; // convert to m/s

  /***
  C. Transform map space data to vehicle space
  ***/
          Eigen::VectorXd   ptsx_veh = Eigen::VectorXd( ptsx.size() );
          Eigen::VectorXd   ptsy_veh = Eigen::VectorXd( ptsx.size() );

          for (unsigned int i = 0;   i < ptsx.size() ;   i++) {
            ptsx_veh(i) = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
            ptsy_veh(i) = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi);
          }

  /***
  D. Calculate polynomial fit
  ***/

          // fit a polynomial to the above x and y coordinates
          int iPoly_ord = 3;
          auto coeffs = polyfit(ptsx_veh, ptsy_veh, iPoly_ord);


  /***
  E. Calculate errors
  ***/

          // calculate the cross track error
          double cte = polyeval(coeffs, 0.0, false) - 0.0;
          // calculate the orientation error
          double epsi = 0.0 - atan(polyeval(coeffs, 0.0, true));

          double steer_value;
          double throttle_value;
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          vector<double> next_x_vals(ptsx_veh.data()+1, ptsx_veh.data() + ptsx_veh.size());
          vector<double> next_y_vals(ptsy_veh.data()+1, ptsy_veh.data() + ptsy_veh.size());

          Eigen::VectorXd state(6);
          state << 0.0, 0.0, 0.0, v, cte, epsi;

  /***
  F. MPC Solve
  ***/
          size_t iIters = 3;
          for (size_t i = 0; i < iIters; i++) {
          //std::cout << "Iteration " << i << std::endl;
          auto vars = mpc.Solve(state, coeffs);
          std::tie(state[0], state[1], state[2], state[3], state[4], state[5], steer_value, throttle_value, mpc_x_vals, mpc_y_vals) = vars;
          }

          steer_value = -1*steer_value/0.436332; //Normalizing steering value and dealing with angle rotation

          //cout<<"steer_value: \t"<<steer_value <<"\n";
          //cout<<"throttle_value: \t"<<throttle_value <<"\n";

  /***
  G. Output to simulator
  ***/

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.

          this_thread::sleep_for(chrono::milliseconds(100));
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
