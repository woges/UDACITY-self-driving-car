#include "vehicle.h"
#include "helper.h"
#include "constants.h"
#include "cost_functions.h"
#include "coordinates.h"
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
                  vector<vector<vector<double>>> gl_vLane_traffic_last){

  vector<double> car_set;
  vector<double> fill_set1 = {-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,9999., -1.};
  vector<double> fill_set2 = {-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-9999., -1.};
  vector<vector<vector<double>>> car_ahead(3), car_behind(3);
  vector<vector<vector<double>>> vAll_Lane_traffic(3);
  vector<int> viCars_ahead(3);

  //gl_vLane_traffic.clear();

  for(int i=0; i<3;i++){
    car_ahead[i].push_back(fill_set1);
    car_behind[i].push_back(fill_set2);
  }

  // check in which lane each car is, sort them and save only the nearest
  // cars behind and ahead, save also the vehicle speed and s-distance in relation to
  // our car, lane and also the "traffic" ahead for each lane
  // sensor fusion = [id, x, y, vx, vy, s, d]
  // expand to [id, x, y, vx, vy, s, d, v, s-dist, lane, traffic ahead]
  for (int i = 0; i < gl_vSensor_fusion.size(); i++){
    // get vehicle set from sensor fusion, calculate values, sort in lanes
    if(gl_vSensor_fusion[i][6]>0){
      car_set = gl_vSensor_fusion[i];
      car_calculate(car_set,own_car);
      vAll_Lane_traffic[int(car_set[9])].push_back(car_set);
      if(car_set[8]>0){
        viCars_ahead[int(car_set[9])]+=1;
      }
    }
  }

  // sort all other cars by lane and than by distance
  for (int k = 0; k < 3; k++){
    for(int p = 0; p < vAll_Lane_traffic[k].size(); p++){
      vector<double> car_temp = vAll_Lane_traffic[k][p];
      if(car_temp[8]>0){
        if(car_temp[8]<car_ahead[k][0][8]){
          car_ahead[k][0] = car_temp;
        }
      }
      else{
        if(car_temp[8]>car_behind[k][0][8]){
          car_behind[k][0] = car_temp;
        }
      }
    }
  }

  // calculate acceleration of those cars
  for (int k = 0; k < 3; k++){
      double ah_acc = 0.;
      double bh_acc = 0.;
      vector<double> car_tmp_ah = car_ahead[k][0];
      vector<double> car_tmp_bh = car_behind[k][0];
      //calculate acc

      if(!gl_vLane_traffic_last[k].empty()){
        if(gl_vLane_traffic_last[k][0][0] == car_tmp_ah[0]){
          ah_acc = ((car_tmp_ah[7] - gl_vLane_traffic_last[k][0][7])/(own_car.dTimestep * own_car.iNum_Steps));
        }
        if(gl_vLane_traffic_last[k][1][0] == car_tmp_bh[0]){
          bh_acc = ((car_tmp_bh[7] - gl_vLane_traffic_last[k][1][7])/(own_car.dTimestep * own_car.iNum_Steps));
        }
      }

      car_tmp_ah.push_back(viCars_ahead[k]);
      car_tmp_ah.push_back(ah_acc);

      car_tmp_bh.push_back(viCars_ahead[k]);
      car_tmp_bh.push_back(bh_acc);

      if(!gl_vLane_traffic[k].empty()){
        gl_vLane_traffic[k][0] = car_tmp_ah;
        gl_vLane_traffic[k][1] = car_tmp_bh;
      }
      else{
        gl_vLane_traffic[k].push_back(car_tmp_ah);
        gl_vLane_traffic[k].push_back(car_tmp_bh);
      }
  }
}

// calculates all additional values for each car
void car_calculate(vector<double> &car_set, Vehicle &own_car){
  // calculate the cars speed
  double speed = sqrt(car_set[3]*car_set[3] + car_set[4]*car_set[4])/2.24; //m/s
  // calculate the cars distance from ego car
  double dist = car_set[5] - own_car.s;
  // acounts for the lap length and back changing of s to 0
  if(dist < -5000.){
    car_set[5] += own_car.dLap_length;
  } else if(dist > 5000.){
    car_set[5] -= own_car.dLap_length;
  }
  dist = car_set[5] - own_car.s;
  // calculate the cars lane
  int lane = static_cast<int> (trunc(car_set[6]/ 4.0));

  car_set.push_back(speed);
  car_set.push_back(dist);
  car_set.push_back(double(lane));
}
vector<vector<double>> PathPlanner(vector<vector<vector<double>>> &gl_vLane_traffic, Vehicle &own_car, int &iNext_Lane,const vector<double>  &map_waypoints_x,const vector<double> &map_waypoints_y,const vector<double> &map_waypoints_s) {
//vector<double> PathPlanner(vector<vector<vector<double>>> &gl_vLane_traffic, Vehicle &own_car, int &iNext_Lane){

  struct constants stConst;
  bool debug_path = false;
  bool debug_pos = false;

  double dDist_Path_ah, dAhead_car_sc, dAhead_car_vc, dTime_futur;
  vector<double> vdOwn_car(3), vdOwn_car_next(2), vdAhead_car(3);
  dTime_futur = own_car.iNum_Steps * own_car.dTimestep;

  if(debug_path){
    cout << "Path Planner start: " << endl;
    cout << "own_car.state: "<< own_car.state << endl;
    cout << "own_car.d: "<< own_car.d << endl;
    cout << "own_car.lane: "<< own_car.lane << endl;
    cout << "iNext_Lane  : "<< iNext_Lane << endl;
    cout << "bLane_Change: "<< own_car.bLane_Change << endl;
  }


// depending on the choosen next state, check if there is a car ahead and when calculate where the car will be after 50x0.02s = 1s
  string check_state = own_car.state;

  if(own_car.bLane_Change){


    // distance to car ahead
    // dDist_Path_ah => calculate distance to ahead car at rest of 50Steps * 0.02s in future
    dAhead_car_sc = gl_vLane_traffic[iNext_Lane][0][5];
    dAhead_car_vc = gl_vLane_traffic[iNext_Lane][0][7];
    // ahead car state at iNext Lane
    if(gl_vLane_traffic[iNext_Lane][0][5] == -1) {
      vdAhead_car[0] = 9999.;
      vdAhead_car[1] = 50./2.24;
    } else {
      vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., 1., 1);
    }
    //#################################

    if(iNext_Lane == own_car.lane && fabs(own_car.d-(4*iNext_Lane+2))<0.5){
      own_car.state = "KL";
      own_car.bLane_Change = false;
      //cout << "Lane changing done !! Switched to KL " << endl;
    } else {
      own_car.state = "CL";
      own_car.bLane_Change = true;
      //cout << "Changing lane !!!!!!!!!!!!!!!!!!!!! \t" << endl;
    }

  } else {
      if(check_state.compare("KL") == 0){
        own_car.state = "KL";
        own_car.bLane_Change = false;
        iNext_Lane = own_car.lane;

        // distance to car ahead
        // dDist_Path_ah => calculate distance to ahead car at rest of 50Steps * 0.02s in future
        dAhead_car_sc = gl_vLane_traffic[iNext_Lane][0][5];
        dAhead_car_vc = gl_vLane_traffic[iNext_Lane][0][7];

        // ahead car state at iNext Lane
        if(gl_vLane_traffic[iNext_Lane][0][5] == -1) {
          vdAhead_car[0] = 9999.;
          vdAhead_car[1] = 50./2.24;
        } else {
          vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., 1., 1);
        }

        //#################################
        if(debug_pos){
          cout << "Path Planner KL:" << endl;
          cout << "own_car.end_path_s: \t" << own_car.end_path_s << "\town_car.v_end_s: \t" << own_car.v_end_s << "\t vdOwn_car a: \t" <<  own_car.a << endl;
          cout << "vdOwn_car s: \t" << vdOwn_car[0] << "\t vdOwn_car v: \t" << vdOwn_car[1] << "\t vdOwn_car a: \t" <<  own_car.a << endl;
          cout << "vdAhead_car s: \t" << vdAhead_car[0] << "\t vdAhead_car v: \t" << vdAhead_car[1]  << endl;
          cout << "dDist_Path_ah: \t" << dDist_Path_ah << "\t vdOwn_car_next v: \t" << vdOwn_car_next[0] << "\t vdOwn_car_next a: \t" <<  vdOwn_car_next[1] << endl;
        }


      }
      else if(check_state.compare("LCL") == 0){
        //cout << "LCL \t" << endl;
        //cout << "Lane changing started !! Switched to Lane Change " << endl;
        own_car.state = "LCL";
        own_car.bLane_Change = true;
        iNext_Lane -=1;

        // distance to car ahead
        // dDist_Path_ah => calculate distance to ahead car at rest of 50Steps * 0.02s in future
        dAhead_car_sc = gl_vLane_traffic[iNext_Lane][0][5];
        dAhead_car_vc = gl_vLane_traffic[iNext_Lane][0][7];

        // ahead car state at iNext Lane
        if(gl_vLane_traffic[iNext_Lane][0][5] == -1) {
          vdAhead_car[0] = 9999.;
          vdAhead_car[1] = 50./2.24;
        } else {
          vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., 1., 1);
        }

        //#################################
      }
      else if(check_state.compare("LCR") == 0){
        //cout << "LCR \t" << endl;
        //cout << "Lane changing started !! Switched to Lane Change " << endl;
        own_car.state = "LCR";
        own_car.bLane_Change = true;
        iNext_Lane +=1;
        // distance to car ahead
        // dDist_Path_ah => calculate distance to ahead car at rest of 50Steps * 0.02s in future
        dAhead_car_sc = gl_vLane_traffic[iNext_Lane][0][5];
        dAhead_car_vc = gl_vLane_traffic[iNext_Lane][0][7];
        // ahead car state at iNext Lane
        if(gl_vLane_traffic[iNext_Lane][0][5] == -1) {
          vdAhead_car[0] = 9999.;
          vdAhead_car[1] = 50./2.24;
        } else {
          vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., 1., 1);
        }

        //#################################

      }
      else if(check_state.compare("DLC") == 0){
        //cout << "DLC \t" << endl;
        //cout << "Lane changing started !! Switched to Lane Change " << endl;
        own_car.state = "DLC";
        own_car.bLane_Change = true;
        iNext_Lane =1;
        // distance to car ahead
        // dDist_Path_ah => calculate distance to ahead car at rest of 50Steps * 0.02s in future
        dAhead_car_sc = gl_vLane_traffic[iNext_Lane][0][5];
        dAhead_car_vc = gl_vLane_traffic[iNext_Lane][0][7];

        // ahead car state at iNext Lane
        if(gl_vLane_traffic[iNext_Lane][0][5] == -1) {
          vdAhead_car[0] = 9999.;
          vdAhead_car[1] = 50./2.24;
        } else {
          vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., 1., 1);
        }

        //#################################

      }
  }

  if(debug_path){
    cout << "Path Planner end: " << endl;
    cout << "own_car.state: "<< own_car.state << endl;
    cout << "own_car.lane: "<< own_car.lane << endl;
    cout << "iNext_Lane  : "<< iNext_Lane << endl;
    cout << "bLane_Change: "<< own_car.bLane_Change << endl;
  }

// Path and Speed Calculation starts here
  vector<vector<double>> next_vals;
  int prev_size = own_car.iNum_rest_steps;
  double dAcc_target, dSpeed_target;

  double car_s = own_car.s;
  if(prev_size > 0)
  {
    car_s = own_car.end_path_s;
  }

  //bool too_close = false;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // later we will interpolate these waypoints with a spline and fill it in with more points that control speed

  vector<double> ptsx;
  vector<double> ptsy;

  // reference x,y, yaw states
  // either we will reference the starting point as where the car is or at the previous paths end point

  double ref_x = own_car.x;
  double ref_y = own_car.y;
  double ref_yaw = deg2rad(own_car.car_yaw);

  //if previous size is almost empty, use the car as starting reference
  if(prev_size < 2)
  {
    //use two points that make the path tangent to the car
    double prev_car_x = own_car.x - cos(own_car.car_yaw);
    double prev_car_y = own_car.y - sin(own_car.car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(own_car.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(own_car.y);
  }
  // use the previous path's end point as starting reference
  else{
    //Redefine reference state as previous path and end point
    ref_x = own_car.previous_path_x[prev_size-1];
    ref_y = own_car.previous_path_y[prev_size-1];

    double ref_x_prev = own_car.previous_path_x[prev_size-2];
    double ref_y_prev = own_car.previous_path_y[prev_size-2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    //Use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s + 30, (2+4*iNext_Lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2+4*iNext_Lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2+4*iNext_Lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for(int i = 0; i < ptsx.size(); i++){
    //shift car reference angle to 0 degree
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  //create a spline
  tk::spline s;

  //set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  //Start with all of the previous path points from last time
  for(int i = 0; i < own_car.previous_path_x.size(); i++){
    next_x_vals.push_back(own_car.previous_path_x[i]);
    next_y_vals.push_back(own_car.previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
  double x_add_on = 0;

//Calculate a target speed and acceleration depending on the distance from the vehicle ahead
  if((vdAhead_car[0] - own_car.end_path_s) <30.){
    // next car is very close, prepare to break hart and lower speed
    dAcc_target = 9.5 / 50.;
    dSpeed_target = min(vdAhead_car[1],stConst.dSPEED_LIMIT );
  } else if ((vdAhead_car[0] - own_car.end_path_s) > 30. && (vdAhead_car[0] - own_car.end_path_s) <60.){
    // next car is more than 30m but less than 60m away => adapt acceleration speed either to the car ahead or to max. possible
    dAcc_target = 9.5 *(vdAhead_car[0] - own_car.end_path_s)/ 30 - 9.5;
    dAcc_target /= 50.;
    dSpeed_target = (stConst.dSPEED_LIMIT - vdAhead_car[1]) * (vdAhead_car[0] - own_car.end_path_s)/ 30 + (2*vdAhead_car[1] - stConst.dSPEED_LIMIT);
  } else {
    // next car is more than 60m away => max. acceleration, max. speed possible
    dAcc_target = 9.5 / 50.;
    dSpeed_target = stConst.dSPEED_LIMIT;
  }
// Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  for(int i = 1; i <= 50 - own_car.previous_path_x.size(); i++){
    // here calculate the next speedstep
    if(own_car.ref_vel > dSpeed_target){
      own_car.ref_vel -= dAcc_target; //9.5m/s²@0.02s
    }
    else if ((dSpeed_target-0.1)< own_car.ref_vel && own_car.ref_vel < (dSpeed_target+0.1)){
      own_car.ref_vel = dSpeed_target;
    } else {
      //don't accelerate too much when changing lane
      if(own_car.bLane_Change) dAcc_target *= 0.8;
      own_car.ref_vel += dAcc_target; //9.5m/s²@0.02s
    }
    //cout << "dSpeed_target: \t " << dSpeed_target << "\t dAcc_target: \t" << dAcc_target << endl;
    //cout << "ref_vel end: " << own_car.ref_vel << endl << endl << endl;
    double N = (target_dist/(0.02*own_car.ref_vel));
// velocity target end
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double dX_ref = x_point;
    double dY_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = (dX_ref*cos(ref_yaw) - dY_ref*sin(ref_yaw));
    y_point = (dX_ref*sin(ref_yaw) + dY_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);


  return next_vals;
}

// checks which trajectory would be best
void BehavioralPlanner(vector<vector<vector<double>>> &gl_vLane_traffic, Vehicle &own_car, int &iNext_Lane){

  int iCurrent_Lane;
  int iSteps = 10;
  int  iHorizon = 5;
  double dTime_futur = double((1.*iHorizon)/iSteps);
  struct constants stConst;

  if(!own_car.bLane_Change){
    vector<double> cost_sum(own_car.vst_Pos_states.size());
    // check a time horizon of 5 seconds, calculate 10 steps, the costs for each step and sum it up
    // take all state into account, here Keep Lane, Lane Change Left, Lane Change Right, Double Lane Change
    for(int p = 0; p < (own_car.vst_Pos_states.size()); p++){
      cost_sum[p] = 0.;
      string check_state = own_car.vst_Pos_states[p];
      if(check_state.compare("KL") == 0){
        // just for debugging purpose
        bool debug_kl = false;

        int iTraffic_ahead;
        double dAhead_car_sc, dAhead_car_vc;
        double dBehind_car_sc, dBehind_car_vc;
        bool side_collision = false;
        vector<double> vdOwn_car(3), vdOwn_car_next(2), vdAhead_car(3), vdBehind_car(3);

        iCurrent_Lane = own_car.lane;
        own_car.lane_calc = iCurrent_Lane;

        if(debug_kl){
          cout << "KL \t" << endl;
          cout << "LCL own_car.lane \t" << own_car.lane << endl;
          cout << "LCL iCurrent_Lane \t" << iCurrent_Lane << endl;
          cout << "\t own_car.lane \t" << own_car.lane << "\t own_car.s \t"<< own_car.s << "\t own_car.v \t" << own_car.v << endl;
          cout << "\t dAhead_car \t" << gl_vLane_traffic[iCurrent_Lane][0][9] << "\t dAhead_car.s \t"<< gl_vLane_traffic[iCurrent_Lane][0][5] << "\t dAhead_car.v \t" << gl_vLane_traffic[iCurrent_Lane][0][7] << endl;
          cout << "\t dBehind_car \t" << gl_vLane_traffic[iCurrent_Lane][1][9] << "\t dBehind_car.s \t"<< gl_vLane_traffic[iCurrent_Lane][1][5] << "\t dBehind_car.v \t" << gl_vLane_traffic[iCurrent_Lane][1][7] << endl << endl;
        }

        // Take only car ahead into account, calculate possible state at different timesteps into future
        own_car.s_calc = own_car.s;
        own_car.v_calc = own_car.v;
        own_car.a_calc = own_car.a;
        dAhead_car_sc = gl_vLane_traffic[iCurrent_Lane][0][5];
        dAhead_car_vc = gl_vLane_traffic[iCurrent_Lane][0][7];
        //dBehind_car_sc = gl_vLane_traffic[iCurrent_Lane][1][5];
        //dBehind_car_vc = gl_vLane_traffic[iCurrent_Lane][1][7];

        for(int k = 0; k <= iSteps; k ++){
          if(own_car.v_calc==0 && own_car.a_calc == 0 && own_car.iNum_rest_steps < 2){
              own_car.a_calc = stConst.dMAX_ACCEL;
              vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
          } else {
            vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
          }
          own_car.a_calc = vdOwn_car[2];

          // ahead car state at
          if(gl_vLane_traffic[iCurrent_Lane][0][5] == -1) { // there is no car in front of us
            vdAhead_car[0] = 9999.;
            vdAhead_car[1] = 50./2.24;
            iTraffic_ahead = 0;
          } else {
            vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., dTime_futur, k);
            iTraffic_ahead = gl_vLane_traffic[iCurrent_Lane][0][10];
          }
          // car behind state: necessary only for lane change
          /*
          if(gl_vLane_traffic[iCurrent_Lane][1][5] == -1) {
            vdBehind_car[0] = -9999.;
            vdBehind_car[1] = 50./2.24;
          } else {
            vdBehind_car = state_at(dBehind_car_sc, dBehind_car_vc, gl_vLane_traffic[iCurrent_Lane][1][11], dTime_futur, k);
          }
          */
          // calculate the distance from the car in front of us
          double veh_dist_ah = vdAhead_car[0] - vdOwn_car[0];

          if(debug_kl){
            cout << "\t own_car.s    \t" << vdOwn_car[0] <<   "\t own_car.v    \t" << vdOwn_car[1] <<   "\t k*dTime_futur \t" << double(k*dTime_futur) << endl;
            cout << "\t dAhead_car s \t" << vdAhead_car[0] << "\t dAhead_car v \t" << vdAhead_car[1] << "\t k*dTime_futur \t" << double(k*dTime_futur) << endl;
            cout << "\t veh_dist_ah s\t" << veh_dist_ah << endl;
          }

          // Calculate the costs for this position (@timestep x) and add them to the total cost of this behavior decision
          cost_sum[p] += cost_function_all(own_car, vdOwn_car, vdAhead_car, veh_dist_ah, iTraffic_ahead, side_collision, vdAhead_car, veh_dist_ah);
          // Calculate new velocity and acceleration depending on all conditions
          vdOwn_car_next = speed_calculation(vdOwn_car, vdAhead_car, veh_dist_ah, own_car.a_calc);
          // Save current values
          own_car.s_calc = vdOwn_car[0];
          own_car.v_calc = vdOwn_car_next[0];
          //if(k == 4) own_car.v_target = own_car.v_calc;
          own_car.a_calc = vdOwn_car_next[1];
          dAhead_car_sc = vdAhead_car[0];
          dAhead_car_vc = vdAhead_car[1];
          //dBehind_car_sc = vdBehind_car[0];
          //dBehind_car_vc = vdBehind_car[1];
          //;
        }
        //cout << "cost_sum[p]: \t" <<cost_sum[p] << endl;
      }
      else if(check_state.compare("LCL") == 0){

        // first check, if not already in the leftmost lane
        if(own_car.lane == 0){
          cost_sum[p] = 9999.;
        } else{
          iCurrent_Lane = own_car.lane - 1;


    //####################################################
          bool debug_lcl = false;
          own_car.lane_calc = iCurrent_Lane;
          int iTraffic_ahead;
          double dAhead_car_sc, dAhead_car_vc;
          double dBehind_car_sc, dBehind_car_vc;
          bool side_collision = true;
          vector<double> vdOwn_car(3), vdOwn_car_next(2), vdAhead_car(3), vdBehind_car(3);


          if(debug_lcl){
            cout << "LCL \t" << endl;
            cout << "LCL own_car.lane \t" << own_car.lane << endl;
            cout << "LCL iCurrent_Lane \t" << iCurrent_Lane << endl;
            cout << "\t own_car.lane \t" << own_car.lane << "\t own_car.s \t"<< own_car.s << "\t own_car.v \t" << own_car.v << endl;
            cout << "\t dAhead_car \t" << gl_vLane_traffic[iCurrent_Lane][0][9] << "\t dAhead_car.s \t"<< gl_vLane_traffic[iCurrent_Lane][0][5] << "\t dAhead_car.v \t" << gl_vLane_traffic[iCurrent_Lane][0][7] << endl;
            cout << "\t dBehind_car \t" << gl_vLane_traffic[iCurrent_Lane][1][9] << "\t dBehind_car.s \t"<< gl_vLane_traffic[iCurrent_Lane][1][5] << "\t dBehind_car.v \t" << gl_vLane_traffic[iCurrent_Lane][1][7] << endl << endl;
          }

          // Take car ahead / behind into account, calculate possible state at different timesteps
          own_car.s_calc = own_car.s;
          own_car.v_calc = own_car.v;
          own_car.a_calc = own_car.a;
          dAhead_car_sc = gl_vLane_traffic[iCurrent_Lane][0][5];
          dAhead_car_vc = gl_vLane_traffic[iCurrent_Lane][0][7];
          dBehind_car_sc = gl_vLane_traffic[iCurrent_Lane][1][5];
          dBehind_car_vc = gl_vLane_traffic[iCurrent_Lane][1][7];

          for(int k = 0; k <= iSteps; k ++){
            // own_car state at timestep
            if(own_car.v_calc==0 && own_car.a_calc == 0 && own_car.iNum_rest_steps < 2){
              own_car.a_calc = stConst.dMAX_ACCEL;
              vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
            } else {
              vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
            }
            own_car.a_calc = vdOwn_car[2];

            // ahead car state at
            if(gl_vLane_traffic[iCurrent_Lane][0][5] == -1) {
              vdAhead_car[0] = 9999.;
              vdAhead_car[1] = 50./2.24;
              iTraffic_ahead = 0;
            } else {
              vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., dTime_futur, k);
              //vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, gl_vLane_traffic[iCurrent_Lane][0][11], dTime_futur, k);
              iTraffic_ahead = gl_vLane_traffic[iCurrent_Lane][0][10];
            }
            // behind car state: necessary only for lane change

            if(gl_vLane_traffic[iCurrent_Lane][1][5] == -1) {
              vdBehind_car[0] = -9999.;
              vdBehind_car[1] = 50./2.24;
            } else {
              vdBehind_car = state_at(dBehind_car_sc, dBehind_car_vc, 0., dTime_futur, k);
              //vdBehind_car = state_at(dBehind_car_sc, dBehind_car_vc, gl_vLane_traffic[iCurrent_Lane][1][11], dTime_futur, k);
            }


            double veh_dist_ah = vdAhead_car[0] - vdOwn_car[0];

            double veh_dist_bh = vdBehind_car[0] - vdOwn_car[0];

            if(debug_lcl){
              cout << "\t own_car.s    \t" << vdOwn_car[0] <<   "\t own_car.v    \t" << vdOwn_car[1] <<   "\t k*dTime_futur \t" << double(k*dTime_futur) << endl;
              cout << "\t vdAhead_car s \t" << vdAhead_car[0] << "\t dAhead_car v \t" << vdAhead_car[1] << "\t k*dTime_futur \t" << double(k*dTime_futur) << endl;
              cout << "\t veh_dist_ah s\t" << veh_dist_ah << endl;
              cout << "\t vdBehind_car s \t" << vdBehind_car[0] << "\t dAhead_car v \t" << vdBehind_car[1] << "\t k*dTime_futur \t" << double(k*dTime_futur) << endl;
              cout << "\t veh_dist_bh s\t" << veh_dist_bh << endl;
            }



            // Calculate the costs for this position and add them to the total cost of this behavior decision
            cost_sum[p] += cost_function_all(own_car, vdOwn_car, vdAhead_car, veh_dist_ah, iTraffic_ahead, side_collision, vdBehind_car, veh_dist_bh);
            // Calculate new velocity and acceleration depending on all conditions
            vdOwn_car_next = speed_calculation(vdOwn_car, vdAhead_car, veh_dist_ah, own_car.a_calc);
            // Save current values
            own_car.s_calc = vdOwn_car[0];
            own_car.v_calc = vdOwn_car_next[0];
            //if(k == 4) own_car.v_target = own_car.v_calc;
            own_car.a_calc = vdOwn_car_next[1];
            dAhead_car_sc = vdAhead_car[0];
            dAhead_car_vc = vdAhead_car[1];
            dBehind_car_sc = vdBehind_car[0];
            dBehind_car_vc = vdBehind_car[1];
            //;
          }
        }
        //cout << "cost_sum[p]: \t" <<cost_sum[p] << endl;
  //####################################################
      }
      else if(check_state.compare("LCR") == 0){

        // fist check, if not already in the rightmost lane
        if(own_car.lane == 2){
          cost_sum[p] = 9999.;
        } else {
          iCurrent_Lane = own_car.lane + 1;

    //####################################################
          bool debug_lcr = false;
          own_car.lane_calc = iCurrent_Lane;
          int iTraffic_ahead;
          double dAhead_car_sc, dAhead_car_vc;
          double dBehind_car_sc, dBehind_car_vc;
          bool side_collision = true;
          vector<double> vdOwn_car(3), vdOwn_car_next(2), vdAhead_car(3), vdBehind_car(3);

          if(debug_lcr){
            cout << "LCR \t" << endl;
            cout << "\t own_car.lane \t" << own_car.lane << "\t own_car.s \t"<< own_car.s << "\t own_car.v \t" << own_car.v << endl;
            cout << "\t dAhead_car \t" << gl_vLane_traffic[iCurrent_Lane][0][9] << "\t dAhead_car.s \t"<< gl_vLane_traffic[iCurrent_Lane][0][5] << "\t dAhead_car.v \t" << gl_vLane_traffic[iCurrent_Lane][0][7] << endl;
            cout << "\t dBehind_car \t" << gl_vLane_traffic[iCurrent_Lane][1][9] << "\t dBehind_car.s \t"<< gl_vLane_traffic[iCurrent_Lane][1][5] << "\t dBehind_car.v \t" << gl_vLane_traffic[iCurrent_Lane][1][7] << endl << endl;
          }

          // Take car ahead / behind into account, calculate possible state at different timesteps
          own_car.s_calc = own_car.s;
          own_car.v_calc = own_car.v;
          own_car.a_calc = own_car.a;
          dAhead_car_sc = gl_vLane_traffic[iCurrent_Lane][0][5];
          dAhead_car_vc = gl_vLane_traffic[iCurrent_Lane][0][7];
          dBehind_car_sc = gl_vLane_traffic[iCurrent_Lane][1][5];
          dBehind_car_vc = gl_vLane_traffic[iCurrent_Lane][1][7];

          for(int k = 0; k <= iSteps; k ++){
            // own_car state at timestep
            if(own_car.v_calc==0 && own_car.a_calc == 0 && own_car.iNum_rest_steps < 2){
              own_car.a_calc = stConst.dMAX_ACCEL;
              vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
            } else {
              vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
            }
            own_car.a_calc = vdOwn_car[2];

            // ahead car state at timestep
            if(gl_vLane_traffic[iCurrent_Lane][0][5] == -1) {
              vdAhead_car[0] = 9999.;
              vdAhead_car[1] = 50./2.24;
              iTraffic_ahead = 0;
            } else {
              vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., dTime_futur, k);
              //vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, gl_vLane_traffic[iCurrent_Lane][0][11], dTime_futur, k);
              iTraffic_ahead = gl_vLane_traffic[iCurrent_Lane][0][10];
            }
            // behind car state: necessary only for lane change

            if(gl_vLane_traffic[iCurrent_Lane][1][5] == -1) {
              vdBehind_car[0] = -9999.;
              vdBehind_car[1] = 50./2.24;
            } else {
              vdBehind_car = state_at(dBehind_car_sc, dBehind_car_vc, 0., dTime_futur, k);
              //vdBehind_car = state_at(dBehind_car_sc, dBehind_car_vc, gl_vLane_traffic[iCurrent_Lane][1][11], dTime_futur, k);
            }


            double veh_dist_ah = vdAhead_car[0] - vdOwn_car[0];
            double veh_dist_bh = vdBehind_car[0] - vdOwn_car[0];


            if(debug_lcr){
              cout << "\t own_car.s    \t" << vdOwn_car[0] <<   "\t own_car.v    \t" << vdOwn_car[1] <<   "\t k*dTime_futur \t" << double(k*dTime_futur) << endl;
              cout << "\t vdAhead_car s \t" << vdAhead_car[0] << "\t dAhead_car v \t" << vdAhead_car[1] << "\t veh_dist_ah \t" << veh_dist_ah << endl;
              cout << "\t vdBehind_car s \t" << vdBehind_car[0] << "\t dAhead_car v \t" << vdBehind_car[1] << "\t veh_dist_bh s \t" << veh_dist_bh << endl;
              cout  << endl;
            }



            // Calculate the costs for this position and add them to the total cost of this behavior decision
            cost_sum[p] += cost_function_all(own_car, vdOwn_car, vdAhead_car, veh_dist_ah, iTraffic_ahead, side_collision, vdBehind_car, veh_dist_bh);
            // Calculate new velocity and acceleration depending on all conditions
            vdOwn_car_next = speed_calculation(vdOwn_car, vdAhead_car, veh_dist_ah, own_car.a_calc);
            // Save current values
            own_car.s_calc = vdOwn_car[0];
            own_car.v_calc = vdOwn_car_next[0];
            //if(k == 4) own_car.v_target = own_car.v_calc;
            own_car.a_calc = vdOwn_car_next[1];
            dAhead_car_sc = vdAhead_car[0];
            dAhead_car_vc = vdAhead_car[1];
            dBehind_car_sc = vdBehind_car[0];
            dBehind_car_vc = vdBehind_car[1];
            //;
          }
        }
        //cout << "cost_sum[p]: \t" <<cost_sum[p] << endl;
  //####################################################
      }
      else if(check_state.compare("DLC") == 0){

        // fist check, in which of the side lanes the car is
        if(own_car.lane == 1){ // if we are in the middle lane there is no double lane change possible
          cost_sum[p] = 9999.;
        } else {
          if(own_car.lane == 0){
        // double lane change to the right/left is calculated and costs are estimated
        // but only a single lane change in this direction is done,
        // then if the left-/rightmost lane is still the cheapest decision -  a further lane change will be done automatically with LCR/LCL
        //(=> the single lane change to the right/left has not the minimum costs!!
        // => so we must force the system to do it, so we can do the next lane change to the cheapest lane!!)
            iCurrent_Lane = own_car.lane + 2;
          } else if (own_car.lane == 2){
            iCurrent_Lane = own_car.lane -2; // double lane change to the left
          }


      //####################################################
          bool debug_dlc = false;
          own_car.lane_calc = iCurrent_Lane;

          bool side_collision = true;


          if(debug_dlc){
            cout << "DLC \t" << endl;
            cout << "\t own_car.lane \t" << own_car.lane << "\t own_car.s \t"<< own_car.s << "\t own_car.v \t" << own_car.v << endl;
            cout << "\t dAhead_car \t" << gl_vLane_traffic[iCurrent_Lane][0][9] << "\t dAhead_car.s \t"<< gl_vLane_traffic[iCurrent_Lane][0][5] << "\t dAhead_car.v \t" << gl_vLane_traffic[iCurrent_Lane][0][7] << endl;
            cout << "\t dBehind_car \t" << gl_vLane_traffic[iCurrent_Lane][1][9] << "\t dBehind_car.s \t"<< gl_vLane_traffic[iCurrent_Lane][1][5] << "\t dBehind_car.v \t" << gl_vLane_traffic[iCurrent_Lane][1][7] << endl << endl;
          }

          int iTraffic_ahead;
          double dAhead_car_sc, dAhead_car_vc;
          double dBehind_car_sc, dBehind_car_vc;
          vector<double> vdOwn_car(3), vdOwn_car_next(2), vdAhead_car(3), vdBehind_car(3);

          // Take car ahead and behind into account, calculate possible state at different timesteps
          own_car.s_calc = own_car.s;
          own_car.v_calc = own_car.v;
          own_car.a_calc = own_car.a;
          dAhead_car_sc = gl_vLane_traffic[iCurrent_Lane][0][5];
          dAhead_car_vc = gl_vLane_traffic[iCurrent_Lane][0][7];
          dBehind_car_sc = gl_vLane_traffic[iCurrent_Lane][1][5];
          dBehind_car_vc = gl_vLane_traffic[iCurrent_Lane][1][7];

          for(int k = 0; k <= iSteps; k ++){
            // own_car state at
            //vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
            if(own_car.v_calc==0 && own_car.a_calc == 0 && own_car.iNum_rest_steps < 2){
              own_car.a_calc = stConst.dMAX_ACCEL;
              vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
            } else {
              vdOwn_car = state_at(own_car.s_calc, own_car.v_calc, own_car.a_calc, dTime_futur, k);
            }
            own_car.a_calc = vdOwn_car[2];


            // ahead car state at
            if(gl_vLane_traffic[iCurrent_Lane][0][5] == -1) {
              vdAhead_car[0] = 9999.;
              vdAhead_car[1] = 50./2.24;
              iTraffic_ahead = 0;
            } else {
              vdAhead_car = state_at(dAhead_car_sc, dAhead_car_vc, 0., dTime_futur, k);
              iTraffic_ahead = gl_vLane_traffic[iCurrent_Lane][0][10];
            }
            // behind car state: necessary only for lane change

            if(gl_vLane_traffic[iCurrent_Lane][1][5] == -1) {
              vdBehind_car[0] = -9999.;
              vdBehind_car[1] = 50./2.24;
            } else {
              vdBehind_car = state_at(dBehind_car_sc, dBehind_car_vc, 0., dTime_futur, k);
            }

            double veh_dist_ah = (vdAhead_car[0] - vdOwn_car[0])*2./3.; // additional safe buffer
            double veh_dist_bh = (vdBehind_car[0] - vdOwn_car[0])*2./3.; // additional safe buffer


            if(debug_dlc){
              cout << "\t own_car.s    \t" << vdOwn_car[0] <<   "\t own_car.v    \t" << vdOwn_car[1] <<   "\t k*dTime_futur \t" << double(k*dTime_futur) << endl;
              cout << "\t vdAhead_car s \t" << vdAhead_car[0] << "\t dAhead_car v \t" << vdAhead_car[1] << "\t veh_dist_ah \t" << veh_dist_ah << endl;
              cout << "\t vdBehind_car s \t" << vdBehind_car[0] << "\t dAhead_car v \t" << vdBehind_car[1] << "\t veh_dist_bh s \t" << veh_dist_bh << endl;
              cout  << endl;
            }



            // Calculate the costs for this position and add them to the total cost of this behavior decision
            cost_sum[p] += cost_function_all(own_car, vdOwn_car, vdAhead_car, veh_dist_ah, iTraffic_ahead, side_collision, vdBehind_car, veh_dist_bh);
            // Calculate new velocity and acceleration depending on all conditions
            vdOwn_car_next = speed_calculation(vdOwn_car, vdAhead_car, veh_dist_ah, own_car.a_calc);
            // Save current values
            own_car.s_calc = vdOwn_car[0];
            own_car.v_calc = vdOwn_car_next[0];
            //if(k == 4) own_car.v_target = own_car.v_calc;
            own_car.a_calc = vdOwn_car_next[1];
            dAhead_car_sc = vdAhead_car[0];
            dAhead_car_vc = vdAhead_car[1];
            dBehind_car_sc = vdBehind_car[0];
            dBehind_car_vc = vdBehind_car[1];
            //;
          }
        }
        //cout << "cost_sum[p]: \t" <<cost_sum[p] << endl;
  //####################################################
      }
    }

    // here name the cheapest decision
    double best_cost = 99999.;
    for(int p = 0; p < cost_sum.size(); p++){
      //cout << "cost_sum: " << p << "   " << cost_sum[p] << "\t" << own_car.vst_Pos_states[p] << endl;
      if(cost_sum[p] < best_cost){
        if(own_car.vst_Pos_states[p] == "DLC"){
          auto k_l = find(own_car.vst_Pos_states.begin(), own_car.vst_Pos_states.end(), "LCL") - own_car.vst_Pos_states.begin();
          auto k_r = find(own_car.vst_Pos_states.begin(), own_car.vst_Pos_states.end(), "LCR") - own_car.vst_Pos_states.begin();
          if(cost_sum[k_l] < 70. or cost_sum[k_r] < 70.){
            best_cost = cost_sum[p];
            own_car.state = own_car.vst_Pos_states[p];
          }
        } else {
          best_cost = cost_sum[p];
          own_car.state = own_car.vst_Pos_states[p];
        }
      }
    }

    if(best_cost>70){
      own_car.state = "KL";
    }
  //cout << "chossen own_car.state: \t" << own_car.state << endl;
  }
}

vector<double> state_at(double s_current, double v_current, double a_current, double t, int k) {
	/*
    Predicts state of vehicle in t seconds (assuming constant velocity)
    */
    struct constants stConst;
    double dA_new;
    dA_new = a_current;
    if(k==0) t = 0.;
    double dV_new = v_current + dA_new *t;
    if (dV_new > stConst.dSPEED_LIMIT){
      dV_new = stConst.dSPEED_LIMIT;
      if(t>0.01){
        dA_new = (dV_new - v_current)/t;
      }else {
        dA_new = 0.;
      }
    }

    double s_new = s_current + dV_new * t + dA_new * t * t / 2;


    return {s_new, dV_new, dA_new};
}


vector<double> speed_calculation(vector<double> vdOwn_car, vector<double> vdAhead_car, double veh_dist, double dOwn_acc){
	/*
    Calculates the new velocity and acceleration depending of the distance between car ahead and own car
    */
    vector<double> vdOwn_car_next(2);
    struct constants stConst;
    // Ahead car is further than 80m away => max speed or max acc
    if(veh_dist > stConst.dFREE_MARGIN){
      if(vdOwn_car[1] >= stConst.dSPEED_LIMIT){
        vdOwn_car_next[0] = stConst.dSPEED_LIMIT;
        vdOwn_car_next[1] = 0.;
      } else {
        vdOwn_car_next[0] = vdOwn_car[1];
        vdOwn_car_next[1] = vdOwn_car[2];
      }
    }
    // Ahead car is between 30m and 80m away
    else if(veh_dist > stConst.dBUFFER_MARGIN && veh_dist <= stConst.dFREE_MARGIN){
      if(vdOwn_car[1] > (vdAhead_car[1] + 3.)){
        vdOwn_car_next[0] = min(stConst.dSPEED_LIMIT, vdOwn_car[1]);
        vdOwn_car_next[1] = min(stConst.dMAX_DECEL/2., dOwn_acc);
      } else if(vdOwn_car[1] <= (vdAhead_car[1]+3.) && vdOwn_car[1] > (vdAhead_car[1]-3.)) {
        vdOwn_car_next[0] = min(stConst.dSPEED_LIMIT, vdOwn_car[1]);
        vdOwn_car_next[1] = 0.;
      } else if(vdOwn_car[1] <= (vdAhead_car[1]-3.)){
        vdOwn_car_next[0] = min(stConst.dSPEED_LIMIT, vdAhead_car[1]);
        vdOwn_car_next[1] = max(stConst.dMAX_ACCEL/2., dOwn_acc);
      }
    }
    // Ahead car is between 15 and 30m away
    else if(veh_dist > stConst.dCOLLISION_MARGIN && veh_dist <= stConst.dBUFFER_MARGIN){
      if(vdOwn_car[1] > (vdAhead_car[1] + 3.)){
        vdOwn_car_next[0] = min(stConst.dSPEED_LIMIT, vdOwn_car[1]);
        vdOwn_car_next[1] = stConst.dMAX_DECEL;
      } else if(vdOwn_car[1] <= (vdAhead_car[1]+3.) && vdOwn_car[1] > (vdAhead_car[1]-3.)){
        vdOwn_car_next[0] = min(stConst.dSPEED_LIMIT, vdOwn_car[1]);
        vdOwn_car_next[1] = stConst.dMAX_DECEL/2.;
      } else if(vdOwn_car[1] <= (vdAhead_car[1]-3.)){
        vdOwn_car_next[0] = min(stConst.dSPEED_LIMIT, vdAhead_car[1]);
        vdOwn_car_next[1] = 0.;
      }
    }
    // Ahead car is less than 15m away
    else if(veh_dist <= stConst.dCOLLISION_MARGIN){
      if(vdOwn_car[1] > vdAhead_car[1]){
        vdOwn_car_next[0] = min(stConst.dSPEED_LIMIT, vdOwn_car[1]);
        vdOwn_car_next[1] = 2*stConst.dMAX_DECEL;
      } else if(vdOwn_car[1] < vdAhead_car[1]){
        vdOwn_car_next[0] = min(stConst.dSPEED_LIMIT, vdAhead_car[1]);
        vdOwn_car_next[1] = stConst.dMAX_DECEL;
      }
    }
  return vdOwn_car_next;
}
