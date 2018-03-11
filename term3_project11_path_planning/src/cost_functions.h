#ifndef COST_FUNCTIONS_H_INCLUDED
#define COST_FUNCTIONS_H_INCLUDED

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

double cost_function_all(Vehicle own_car, vector<double> vdOwn_car, vector<double> vdAhead_car, double veh_dist_ah, int iTraffic_ahead,
                         bool side_collision, vector<double> vdBehind_car, double veh_dist_bh);

double cf_lane(struct constants stConst, int iCur_Lane);

double cf_dist(double veh_dist_ah);

double cf_speed_ahead(struct constants stConst, vector<double> vdAhead_car);

double cf_speed_own(struct constants stConst, vector<double> vdOwn_car);

double cf_side_collision(struct constants stConst, double veh_dist_ah, double veh_dist_bh);

double cf_side_buffer(struct constants stConst, double veh_dist_ah, double veh_dist_bh);

#endif // COST_FUNCTIONS_H_INCLUDED
