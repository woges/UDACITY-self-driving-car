#ifndef PID_H
#define PID_H

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <stdlib.h>
#include <numeric>
using namespace std;
using std::vector;

class PID {
public:
  /*
  * Errors
  */
  double d_p_error;
  double d_i_error;
  double d_d_error;

  /*
  * Coefficients
  */
  double d_Kp;
  double d_Kd;
  double d_Ki;


  vector<double> vc_dKparam;
  vector<double> vc_dKtune;
  double norm_steer_value, dN_min_Tol;
  int  iTwd_iter, iN_max_steps, i_iter, k;

  double d_total_error;
  double d_best_error;

  double d_abs_error;
  double d_squared_error;
  double d_mean_squared_error;

  int i_n_steps;
  int iPID_opt;
  int iSwitch_;
  double d_pre_cte;
  double d_steer_max;
  double dTol;

  // save best case
  double d_Kp_best;
  double d_Kd_best;
  double d_Ki_best;

  double d_Speed_sum;

	// Flag, if controller is initialized
	bool is_initialized;
  bool do_twiddle;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
   void Init(double Kp, double Kd , double Ki);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  /*
  * Twiddle
  */
  void Twiddle_param();

  /*
  * Restart Twiddle-loop
  */
  void Twiddle_reset();
};

#endif /* PID_H */
