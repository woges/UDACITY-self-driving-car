#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(){
    this->x_last =  0.;
    this->y_last =  0.;
    this->s_last =  0.;
    this->d_last =  0.;
    this->lane = 1;
    this->car_yaw_last =  0.;
    this->v_last =  0.;
    this->a_last =  -99.;
    this->ref_vel = 0.0;

    this->x = 0.;
    this->y = 0.;
    this->s = 0.;
    this->d = 0.;
    this->lane = 1;
    this->car_yaw =  0.;
    this->v =  0.;
    this->a = 0.;

    this->bLane_Change = false;
}

Vehicle::~Vehicle() {}


void Vehicle::vehicle_set(double x, double y, double s, double d, double car_yaw, double car_speed,
                          vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                          double end_path_d) {

    this->x_last = this->x;
    this->y_last = this->y;
    this->s_last = this->s;
    this->d_last = this->d;
    this->car_yaw_last = this->car_yaw;
    this->v_last = this->v;
    this->a_last = this->a;
    this->lane_last = this->lane;

    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->lane = static_cast<int> (trunc(this->d/ 4.0));
    this->car_yaw = car_yaw;
    double real_timestep = double(this->dTimestep*this->iNum_Steps);
    this->v = car_speed/2.24; //m/s


    double dPath_dist;
    double dPath_time;
    double dAcc2;

    if(this->a_last != -99. && this->v_last != 0.){
      if(real_timestep > 0.01){
          if(this->v !=0){
             dAcc2 = (this->v - this->v_last)/real_timestep;
            this->a = dAcc2;
          }
      } else{
          this->a = 0.;
      }
    }

    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;

}
