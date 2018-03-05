#define _USE_MATH_DEFINES

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <stdlib.h>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
//int main(int argc, const char *argv[])
int main()
{
  uWS::Hub h;

  PID pidStear;
  PID pidSpeed;



  h.onMessage([&pidStear,&pidSpeed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

// choose wether constant speed or PID controlled speed
    bool speed_const = false;
// select max speed for PID controller or constant speed value
    double dSpeed_max = 95.;
    double dSpeed_const = 0.75;

    // Initialize the pid variable.

    if (!pidStear.is_initialized) {
      // start parameter
      pidStear.vc_dKparam = {3.2, 170, 0.0002} ;
      //pidStear.vc_dKparam = {2.0, 80.0, 0.0002} ;
      pidStear.vc_dKtune = { 1.0, 20.0, 0.0005} ;
      pidStear.do_twiddle = false;
      pidStear.Init(pidStear.vc_dKparam[0], pidStear.vc_dKparam[1], pidStear.vc_dKparam[2]);
      //cout << "\n start steer parameter \n" << pidStear.d_Kp << "\t" << pidStear.d_Kd << "\t" << pidStear.d_Ki << "\t" << pidStear.iPID_opt << "\n";
    }

    if (!pidSpeed.is_initialized) {
      // start parameter
      pidSpeed.vc_dKparam = { 18.0, 20.0, 0.002} ;
      pidSpeed.vc_dKtune = { 1., 1.0, 0.0005};
      pidSpeed.do_twiddle = false;
      pidSpeed.Init(pidSpeed.vc_dKparam[0], pidSpeed.vc_dKparam[1], pidSpeed.vc_dKparam[2]);
      //cout << "\n start speed parameter \n" << pidSpeed.d_Kp << "\t" << pidSpeed.d_Kd << "\t" << pidSpeed.d_Ki << "\t" << pidSpeed.iPID_opt << "\n";
    }
   // Init


    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {

        auto j = json::parse(s);
        // do steer twiddle when number of steps is reached (e.g. one lap)
        if(pidStear.do_twiddle && pidStear.i_n_steps == pidStear.iPID_opt){

          std::string event = j[0].get<std::string>();
          if (event == "telemetry") {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());

             // update errors and calculate the steer_value
            cte = pow(cte,2)*cte/fabs(cte);
            pidStear.UpdateError(cte);
            // twiddle loop here
            pidStear.Twiddle_param();

            pidStear.i_n_steps = 1;
            pidStear.norm_steer_value = 0;
            cte = 0.;

            // reset vehicle to start point and condition
            json msgJson;
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }
        // do speed twiddle when number of steps is reached (e.g. one lap)
        } else if(pidSpeed.do_twiddle && pidSpeed.i_n_steps == pidSpeed.iPID_opt){

          std::string event = j[0].get<std::string>();
          if (event == "telemetry") {
            // j[1] is the data JSON object
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());
//            double dSpeed_max = 100.;

            pidSpeed.d_Speed_sum +=speed;

            double ct_error;
            // calculate a cte with speed difference and a function pow(x,4)=> forcing lower speed at higher angles
            ct_error = -1*pow((speed - dSpeed_max)/10.,2) + pow(fabs(angle)/4.7,4)/15.;
            pidSpeed.UpdateError(ct_error);
            pidSpeed.d_mean_squared_error = 100. - pidSpeed.d_Speed_sum  / pidSpeed.i_n_steps;
            //cout<<"d_mean_squared_error d_Speed_sum: \t" <<  pidSpeed.d_mean_squared_error <<"\n";
            // twiddle loop here
            pidSpeed.Twiddle_param();

            pidSpeed.i_n_steps = 1;
            pidSpeed.norm_steer_value = 0;
            speed = 0.;
            angle = 0.;
            // reset vehicle to start point and condition
            json msgJson;
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }
        // no twiddle, just cruise around
        } else {
          std::string event = j[0].get<std::string>();
          if (event == "telemetry") {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            double steer_value, speed_value;
//            double dSpeed_max = 100.;

            pidSpeed.d_Speed_sum +=speed;
            cte = pow(cte,2)*cte/fabs(cte);
             // update errors and calculate the steer_value
            pidStear.UpdateError(cte);
            steer_value =pidStear.TotalError();
            // normalize steer value to [-1,1]
            pidStear.norm_steer_value = steer_value/(pidStear.d_steer_max) ;
            if(fabs(pidStear.norm_steer_value)>1.0){
              pidStear.norm_steer_value /=fabs(pidStear.norm_steer_value);
            }
            // speed value
            double ct_error;
            ct_error = -1*pow((speed - dSpeed_max)/10.,2) + pow(fabs(angle)/4.7,4)/15.;

            // update speed errors
            pidSpeed.UpdateError(ct_error);
            speed_value =pidSpeed.TotalError();

            pidSpeed.norm_steer_value = speed_value;
            // normalize speed value to [0,1]
            if(pidSpeed.norm_steer_value < 0.0 ){
              pidSpeed.norm_steer_value = 0.0;
            } else if (pidSpeed.norm_steer_value > dSpeed_max) {
              pidSpeed.norm_steer_value = dSpeed_max/100.;
            }else {
              pidSpeed.norm_steer_value = pidSpeed.norm_steer_value / 100.;
            }
            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << pidStear.norm_steer_value << "  Roh steering " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = pidStear.norm_steer_value;
            if(speed_const){
              msgJson["throttle"] = dSpeed_const;
            }else {
              msgJson["throttle"] =  pidSpeed.norm_steer_value;
            }

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
