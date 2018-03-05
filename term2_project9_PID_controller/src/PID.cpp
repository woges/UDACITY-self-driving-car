#include "PID.h"


using namespace std;

/*
* TODO: Complete the PID class.
*/

//PID::PID() {}
PID::PID() : is_initialized(false) {};
PID::~PID() {}

void PID::Init( double Kp, double Kd, double Ki) {

	// Flag, if controller is initialized
	is_initialized = true;

  d_p_error = 0.;
  d_i_error = 0.;
  d_d_error = 0.;

  d_Kp = Kp;
  d_Kd = Kd;
  d_Ki = Ki;

  d_total_error = 0.;

  // normalize steering angle (-25° / 25°)
  d_steer_max = 25.0;

  // Twiddle parameter
  dTol = 0.1;
  iSwitch_ = 0;
  d_abs_error = 0.;
  d_squared_error = 0.;
  d_mean_squared_error = 0.;
  d_best_error = std::numeric_limits<double>::max();
  i_n_steps = 1;


	//do_twiddle = true;
  iPID_opt = 4000;
  iN_max_steps = 4000;
  dN_min_Tol = 0.001;
  iTwd_iter= 1;
  i_iter = 2;
  k = 0;

  d_Kp_best = Kp;
  d_Kd_best = Kd;
  d_Ki_best = Ki;

  d_Speed_sum = 0.0;


}

void PID::UpdateError(double cte) {

  if (i_n_steps == 1) {
      // to get correct initial d_error
          d_pre_cte = cte;
  }
  d_p_error = cte;
  d_d_error = cte - d_pre_cte;
  d_i_error += cte;

  d_pre_cte = cte;

  d_abs_error +=fabs(cte) ;
  d_squared_error +=pow(cte,2);
  d_mean_squared_error = d_squared_error / i_n_steps;

  i_n_steps +=1;
}

double PID::TotalError() {
 d_total_error = - d_Kp * d_p_error
                 - d_Kd * d_d_error
                 - d_Ki * d_i_error;
  return d_total_error;
}

  /*
  * Twiddle
  */
  void PID::Twiddle_param(){
    if(accumulate(begin(vc_dKtune), end(vc_dKtune), 0.0, plus<double>())>dTol){

      if(d_mean_squared_error < d_best_error){
        d_best_error = d_mean_squared_error;
        d_Kp_best =  d_Kp;
        d_Kd_best =  d_Kd;
        d_Ki_best =  d_Ki;
        iSwitch_ =1;
      } else {
        iSwitch_ +=2;
      }

      cout<<"\n Iteration " <<  iTwd_iter << "\t" << "best error = " << d_best_error <<"\n";
      cout<<"Best parameter set: \n ";
      cout<< d_Kp_best <<"\t"<< d_Kd_best << "\t"<<d_Ki_best <<"\n";
      cout<<"Actual parameter set: \n ";
      cout<< d_Kp <<"\t"<< d_Kd << "\t"<<d_Ki <<"\n";
      cout<< d_abs_error <<"\t"<< d_squared_error << "\t"<<d_mean_squared_error <<"\n";

      switch(iSwitch_)
      {
        case 1:
          k = i_iter%3;
          vc_dKtune[k] *=1.1;
          iSwitch_ = 0;
          iTwd_iter += 1;
          i_iter +=1;
          k = i_iter%3;
          vc_dKparam[k]+=vc_dKtune[k];
          Twiddle_reset();
          break;
        case 2:
          k = i_iter%3;
          vc_dKparam[k]-=2*vc_dKtune[k];
          Twiddle_reset();
          iTwd_iter += 1;
          break;
        case 4:
          k = i_iter%3;
          vc_dKparam[k]+=vc_dKtune[k];
          vc_dKtune[k] *=0.9;
          iSwitch_ = 0;
          iTwd_iter += 1;
          i_iter +=1;
          k = i_iter%3;
          vc_dKparam[k]+=vc_dKtune[k];
          Twiddle_reset();
          break;
      }
    } else {
      if (iPID_opt<iN_max_steps && dTol>dN_min_Tol){
        iPID_opt  +=1000;
        dTol /=10;
        double dTune = dTol*100. ;
        vc_dKtune = {dTune, dTune, dTune};
        cout<< vc_dKparam[0] <<"\t" << vc_dKparam[1] <<"\t" << vc_dKparam[2] <<"\n" ;
        Twiddle_reset();
        i_iter = 2;
        iTwd_iter += 1;
      } else {

        cout<<"\n Iteration " <<  iTwd_iter << "\t" << "best error = " << d_best_error <<"\n";
        cout<<"\n Best Parameters  Kp: " << d_Kp_best <<"\t Kd: "<< d_Kd_best << "\t  Kd: "<<d_Ki_best <<"\n";
        do_twiddle = false;
        exit(0);
      }
    }
  }

  /*
  * Restart Twiddle-loop
  */
  void PID::Twiddle_reset(){
    cout<<"Twiddle_reset \n";
    d_Kp =  vc_dKparam[0];
    d_Kd =  vc_dKparam[1];
    d_Ki =  vc_dKparam[2];
    d_i_error = 0.;
    d_abs_error =0.;
    d_squared_error =0.;
    d_mean_squared_error = 0.;
    norm_steer_value = 0.;
    d_Speed_sum = 0.0;
   }
