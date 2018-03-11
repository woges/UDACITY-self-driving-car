#ifndef COORDINATES_H_INCLUDED
#define COORDINATES_H_INCLUDED

#include "vehicle.h"
#include "cost_functions.h"
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

double distance(double x1, double y1, double x2, double y2);

constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif // COORDINATES_H_INCLUDED
