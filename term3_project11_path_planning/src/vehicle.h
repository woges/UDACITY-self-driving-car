#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;
class Vehicle;

class Vehicle {
public:
  int lane;

  double x;
  double y;
  double s;
  double d;
  double car_yaw;
  double v;
  double a;
  //double car_speed;
  double ref_vel;

  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;

  int lane_last;

  double x_last;
  double y_last;
  double s_last;
  double d_last;
  double car_yaw_last;
  double v_last;
  double a_last;

  double s_calc;
  double v_calc;
  double a_calc;
  int lane_calc;

  double v_target;
  double v_end_s;

  int iNum_Steps;
  int iNum_rest_steps;

  double dLap_length = 6945.554; //m
  double dTimestep = 0.02; // s

  string state;

  vector<string> vst_Pos_states = {"LCL", "LCR","KL","DLC"};
  bool bLane_Change;

  /**
  * Constructor
  */
  Vehicle();


  /**
  * Destructor
  */
  virtual ~Vehicle();

  void vehicle_set(double x, double y, double s, double d, double car_yaw, double car_speed,
                          vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                          double end_path_d);


};

#endif
