#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(4);

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 0.1, 0, 0, 0,
             0, 0.1, 0, 0,
             0, 0, 9, 0,
             0, 0, 0, 20;
	//measurement matrix
	H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

	//set the acceleration noise components
	noise_ax = 9;
	noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  const double  f_PI=3.14159265358979f;
  bool sensor = true;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double fRho;
      double fTheta;
      double fRho_d;
      double fpx_;
      double fpy_;
      double fvx_;
      double fvy_;

      fRho = measurement_pack.raw_measurements_[0];
      fTheta = measurement_pack.raw_measurements_[1];
      fRho_d = measurement_pack.raw_measurements_[2];

      if(fTheta < (-1*f_PI)){
        while(fTheta<f_PI){
          fTheta = fTheta + (2*f_PI);
        }
      }
      if(fTheta > f_PI){
        while(fTheta > -1*f_PI){
          fTheta = fTheta - (2*f_PI);
        }
      }

      fpx_ = fRho * sin(fTheta);
      fpy_ = fRho * cos(fTheta);
      fvx_ = fRho_d * sin(fTheta);
      fvy_ = fRho_d * cos(fTheta);

      ekf_.x_ << fpx_, fpy_, fvx_, fvy_;
      previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      previous_timestamp_ = measurement_pack.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  double dt_ = (measurement_pack.timestamp_ - previous_timestamp_)/ 1000000.0;	//dt_ - expressed in seconds

  previous_timestamp_ = measurement_pack.timestamp_;
	// Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt_, 0,
             0, 1, 0, dt_,
             0, 0, 1, 0,
             0, 0, 0, 1;
	//Set the process covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);
	double dt4 = pow(dt_,4)/4.;
	double dt3 = pow(dt_,3)/2.;
	double dt2 = pow(dt_,2);
	ekf_.Q_ << dt4*noise_ax, 0, dt3*noise_ax, 0,
			  0, dt4*noise_ay, 0, dt3*noise_ay,
			  dt3*noise_ax, 0, dt2*noise_ax, 0,
			  0, dt3*noise_ay, 0, dt2*noise_ay;


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR and sensor==true) {
    // Radar updates
      ekf_.R_ = MatrixXd(3, 3);
      ekf_.R_ = R_radar_;
      ekf_.H_ = MatrixXd(3, 4);

      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER and sensor==true) {
      ekf_.R_ = MatrixXd(2, 2);
      ekf_.H_  = MatrixXd(2, 4);

      // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;

      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
/*  cout << "x_ = \n" << ekf_.x_ << endl;
  cout << "P_ = \n" << ekf_.P_ << endl;
  cout << "\n\n";
*/
}
