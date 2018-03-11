#include "vehicle.h"
#include "helper.h"
#include "constants.h"
#include "cost_functions.h"
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

// cost functions

double cost_function_all(Vehicle own_car, vector<double> vdOwn_car, vector<double> vdAhead_car, double veh_dist_ah, int iTraffic_ahead,
                         bool side_collision, vector<double> vdBehind_car, double veh_dist_bh){

  bool bDebug_Cost = false;

  struct constants stConst;
  double dTotal_Cost = 0.;
  double dCost_Collision = 0.;
  double dCost_Buffer = 0.;
  vector<double> weights = {0.5, 3., 1., 0., 3., 1000., 5.};


  double dCost_Lane = weights[0] * cf_lane(stConst, own_car.lane_calc); //* cf_lane(stConst, own_car.lane_calc); // costs not being in the middle lane
  double dCost_Dist_Ahead = weights[1] * cf_dist(veh_dist_ah); // costs for being near a car in the same lane
  double dCost_Speed_Ahead = weights[2] * cf_speed_ahead( stConst, vdAhead_car); // costs for driving in a lane with a slow car ahead
  double dCost_Traffic_Ahead =  weights[3] * iTraffic_ahead; // cost for being in a lane with a lot of traffic in front of us
  double dCost_Own_Speed =  weights[4] * cf_speed_own( stConst, vdOwn_car); // cost for not driving with max. speed
  if(side_collision){
    dCost_Collision =  weights[5] * cf_side_collision( stConst, veh_dist_ah, veh_dist_bh); // cost for colliding with other cars when changing lane
    dCost_Buffer =  weights[6] * cf_side_buffer( stConst, veh_dist_ah, veh_dist_bh); // cost for being very near to other cars when changing lane
  }
  dTotal_Cost = dCost_Lane + dCost_Dist_Ahead + dCost_Speed_Ahead + dCost_Traffic_Ahead +  dCost_Own_Speed + dCost_Collision + dCost_Buffer;

  if(bDebug_Cost){
    cout << "dCost_Lane : \t" << dCost_Lane << "dCost_Dist_Ahead : \t" << dCost_Dist_Ahead << "dCost_Speed_Ahead : \t" << dCost_Speed_Ahead <<
    "dCost_Traffic_Ahead : \t" << dCost_Traffic_Ahead << "dCost_Own_Speed : \t" << dCost_Own_Speed << "dCost_Collision : \t" << dCost_Collision <<
    "dCost_Buffer : \t" << dCost_Buffer << endl;
    cout << "dTotal_Cost step: \t" << dTotal_Cost << endl << endl;
  }


  return dTotal_Cost;
}

double cf_lane(struct constants stConst, int iCur_Lane){

  double dLane_Cost = 0.;
  //Calculate lane costs, we prefer to stay in the middle lane
  dLane_Cost = (fabs(stConst.iGOAL_LANE - iCur_Lane));
  //cout << "dLane_Cost: \t" << dLane_Cost << endl;
  return dLane_Cost;
}

double cf_dist(double veh_dist_ah){

  double dDist_Cost = 0.;
  //Calculate distance costs, we prefer to stay in a lane with cars at a far distance from us
  dDist_Cost = min(3.1/(1+exp(0.05*veh_dist_ah)), 1.);
  //cout << "dDist_Cost: \t" << dDist_Cost << endl;
  return dDist_Cost;
}

double cf_speed_ahead(struct constants stConst, vector<double> vdAhead_car){

  double dSpeed_ah_Cost = 0.;
  //Calculate speed costs, we prefer to stay in a lane with cars at a higher speed in front of us
  dSpeed_ah_Cost = (stConst.dSPEED_LIMIT - min(stConst.dSPEED_LIMIT, vdAhead_car[1]))/ stConst.dSPEED_LIMIT;
  //cout << "dSpeed_ah_Cost: \t" << dSpeed_ah_Cost << endl;
  return dSpeed_ah_Cost;
}

double cf_speed_own(struct constants stConst, vector<double> vdOwn_car){

  double dSpeed_own_Cost = 0.;
  //Calculate speed costs, we prefer to drive at 50mph
  dSpeed_own_Cost = (stConst.dSPEED_LIMIT - min(stConst.dSPEED_LIMIT, vdOwn_car[1]))/ stConst.dSPEED_LIMIT;
  //cout << "dSpeed_own_Cost: \t" << dSpeed_own_Cost << endl;
  return dSpeed_own_Cost;
}

double cf_side_collision(struct constants stConst, double veh_dist_ah, double veh_dist_bh){

  double dCollision_Cost = 0.;
  //Calculate speed costs, we prefer to have no collision
  if(fabs(veh_dist_ah) < stConst.dCOLLISION_MARGIN) dCollision_Cost += 1.;
  if(fabs(veh_dist_bh) < stConst.dCOLLISION_MARGIN) dCollision_Cost += 1.;
  //cout << "dCollision_Cost: \t" << dCollision_Cost << endl;
  return dCollision_Cost;
}


double cf_side_buffer(struct constants stConst, double veh_dist_ah, double veh_dist_bh){

  double dBuffer_Cost = 0.;
  //Calculate speed costs, we prefer to have a greater distance than the buffer distance
  if(fabs(veh_dist_ah) < stConst.dBUFFER_MARGIN * 0.75) dBuffer_Cost += 1.;
  if(fabs(veh_dist_bh) < stConst.dBUFFER_MARGIN * 0.75) dBuffer_Cost += 1.;
  //if(fabs(veh_dist_bh) < stConst.dBUFFER_MARGIN/2.) dBuffer_Cost += 1.;
  //cout << "dBuffer_Cost: \t" << dBuffer_Cost << endl;
  return dBuffer_Cost;
}
