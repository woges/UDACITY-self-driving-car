#ifndef HELPER_H_INCLUDED
#define HELPER_H_INCLUDED

#include "vehicle.h"
#include "helper.h"
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <map>
#include <string>
#include <iterator>
#include <algorithm>


void Lane_Traffic(vector<vector<double>> &gl_vSensor_fusion, int &gl_iSensor_fusion_size,
                  vector<vector<vector<double>>> &gl_vLane_traffic, Vehicle &own_car,
                  vector<vector<vector<double>>> gl_vLane_traffic_last);

void car_calculate(vector<double> &car_set, Vehicle &own_car);

//vector<double> PathPlanner(vector<vector<vector<double>>> &gl_vLane_traffic, Vehicle &own_car, int &iNext_Lane);

vector<vector<double>> PathPlanner(vector<vector<vector<double>>> &gl_vLane_traffic, Vehicle &own_car, int &iNext_Lane,const vector<double>  &map_waypoints_x,const vector<double> &map_waypoints_y,const vector<double> &map_waypoints_s);

void BehavioralPlanner(vector<vector<vector<double>>> &gl_vLane_traffic, Vehicle &own_car, int &iNext_Lane);

vector<double> state_at(double s_current, double v_current, double a_current, double t, int k) ;

vector<double> speed_calculation(vector<double> vdOwn_car, vector<double> vdAhead_car, double veh_dist, double dOwn_acc);

#endif // HELPER_H_INCLUDED
