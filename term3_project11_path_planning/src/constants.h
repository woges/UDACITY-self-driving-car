#ifndef CONSTANTS_H_INCLUDED
#define CONSTANTS_H_INCLUDED



#endif // CONSTANTS_H_INCLUDED

struct constants {

//Add all necessary data and constraints as well as variables here
  int  iNUM_LANES = 3;
  int  iGOAL_LANE = 1;

  double dSPEED_LIMIT = 49.5/2.24; //m/s
  double dMAX_ACCEL = 9.5;//m/s²
  double dMAX_DECEL = -9.5;//m/s²

  double dCOLLISION_MARGIN = 15.0; //m
  double dBUFFER_MARGIN = 30.0; //m
  double dFREE_MARGIN = 80.0; //m

  double dGOAL_S = 30.0;
  double dANCHOR_POINTS = 30.0; //m

  double dTIME_STEP = 0.02; // s
  double dLAP_LENGTH = 6945.554; //m

};
