#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#define eps 0.001
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  Complete the initialization. See ukf.h for other member properties.
  */

  is_initialized_ = false;
  //set state dimension
  n_x_ = 5;
  //set augmented dimension
  n_aug_ = 7;
  // set measurement dimension radar
  n_zr = 3;
  // set measurement dimension lidar
  n_zl = 2;
  //define spreading parameter
  lambda_ = 3 - n_aug_;

  // initialize counter for measurement steps
  iCount = 1;

  //define NIS Lidar
  NIS_laser_ = 0.;

  //define NIS Radar
  NIS_radar_ = 0.;

  // initial noise covariance matrix
  Q_ = MatrixXd::Identity(n_aug_-n_x_, n_aug_-n_x_);
  Q_(0,0) = std_a_*std_a_;
  Q_(1,1) = std_yawdd_*std_yawdd_;

  // initial radar measurement noise covariance matrix
  R_ = MatrixXd::Identity(n_zr,n_zr);
  R_(0,0) = std_radr_*std_radr_;
  R_(1,1) = std_radphi_*std_radphi_;
  R_(2,2) = std_radrd_*std_radrd_;

  // initial lidar measurement noise covariance matrix
  L_ = MatrixXd::Identity(n_zl,n_zl);
  L_(0,0) = std_laspx_*std_laspx_;
  L_(1,1) = std_laspy_*std_laspy_;

  // initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initial Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  //set weights
  weights_.setOnes();
  weights_ = weights_ * (1.0/(2.0*(lambda_+n_aug_)));
  weights_(0)=(lambda_/(lambda_+n_aug_));

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package){ //, const VectorXd &ground_truth) {
  // const double  f_PI=3.14159265358979f;
  // bool sensor = true;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ukf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Convert radar from polar to cartesian coordinates.
    */
    // first measurement
    x_ << 1, 1, 1, 1,1;

    double fv_;
    double fPhi_;
    double fPhid_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double fRho;
      double fTheta;
      double fRho_d;
      double fpx_;
      double fpy_;

      fRho = meas_package.raw_measurements_(0);
      fTheta = meas_package.raw_measurements_(1);
      fRho_d = meas_package.raw_measurements_(2);

      fpx_ = fRho * cos(fTheta);
      fpy_ = fRho * sin(fTheta);
      fv_ = 0.0;
      fPhi_ = 0.0;
      fPhid_ = 0.0;
      /**
      Initialize state.
      */
      x_ << fpx_, fpy_, fv_, fPhi_, fPhid_;
      P_ = MatrixXd::Identity(n_x_,n_x_);
      P_(0,0) = 0.09;
      P_(1,1) = 0.09;
      /*P_(2,2) = 2.0;
      P_(3,3) = 2.0;
      P_(4,4) = 2.0;*/

      previous_timestamp_ = meas_package.timestamp_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      fv_ = 0.0;
      fPhi_ = 0.0;
      fPhid_ = 0.0;
      //set the state with the initial location and zero velocity
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], fv_, fPhi_, fPhid_;

      P_ = MatrixXd::Identity(n_x_,n_x_);
      P_(0,0) = 0.0225;
      P_(1,1) = 0.0225;
      P_(2,2) = 0.35;
      P_(3,3) = 0.35;
      P_(4,4) = 0.35;

      previous_timestamp_ = meas_package.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }


  double dt_ = (meas_package.timestamp_ - previous_timestamp_)/ 1000000.0;	//dt_ - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  if(dt_>eps){

    Prediction(dt_);
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_==true) {
    // Radar updates
    UpdateRadar(meas_package);

  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_==true) {
    // Laser updates
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

/*******************************************************************************
 * Generate augmented sigma points
 ******************************************************************************/

  //create augmented mean state
  x_aug << x_, 0, 0;

  //create augmented covariance matrix
  P_aug << P_, MatrixXd::Zero(n_x_, n_aug_-n_x_), MatrixXd::Zero(n_aug_-n_x_, n_x_), Q_;

  //create square root matrix of P_aug
  MatrixXd A_aug = P_aug.llt().matrixL();
  //create augmented sigma points

  Xsig_aug.col(0) = x_aug;

  for(int i=0;i<(n_aug_);i++){
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * A_aug.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A_aug.col(i);
  }

/*******************************************************************************
 * sigma point prediction
 ******************************************************************************/

  VectorXd state_t_ = VectorXd(n_x_);
  VectorXd noise_t_ = VectorXd(n_x_);
  VectorXd xaug_ = VectorXd(n_aug_);

  for(int k=0;k<(2*n_aug_+1);k++){
    xaug_ = Xsig_aug.col(k);
    x_= xaug_.head(n_x_);
    if(fabs(xaug_(4))<0.001){
      state_t_(0)=xaug_(2)*cos(xaug_(3))*delta_t;
      state_t_(1)=xaug_(2)*sin(xaug_(3))*delta_t;
      state_t_(2)=0;
      state_t_(3)=0;
      state_t_(4)=0;
    }
    else {
      state_t_(0)=xaug_(2)/xaug_(4)*(sin(xaug_(3)+xaug_(4)*delta_t)-sin(xaug_(3)));
      state_t_(1)=(xaug_(2)*(-cos(xaug_(3)+xaug_(4)*delta_t)+cos(xaug_(3))))/xaug_(4);
      state_t_(2)=0;
      state_t_(3)=xaug_(4)*delta_t;
      state_t_(4)=0;
    }
    noise_t_(0)= 0.5*delta_t*delta_t*cos(xaug_(3))*xaug_(5);
    noise_t_(1)= 0.5*delta_t*delta_t*sin(xaug_(3))*xaug_(5);
    noise_t_(2)= delta_t*xaug_(5);
    noise_t_(3)= 0.5*delta_t*delta_t*xaug_(6);
    noise_t_(4)= delta_t*xaug_(6);

    Xsig_pred_.col(k) = x_ + state_t_ +noise_t_;

  }
/*******************************************************************************
 * predict mean and covariance
 ******************************************************************************/

  VectorXd x_diff= VectorXd(n_x_);
  //predict state mean
  x_ = Xsig_pred_*weights_;
  //predict state covariance matrix
  P_.fill(0.0);
  for(int k=0; k<(2 * n_aug_ + 1); k++){
      //state difference
      x_diff = (Xsig_pred_.col(k) - x_);
      // angle normalization
      NormalizeAngle(x_diff(3));

      P_ = P_ + weights_(k)* x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  */
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_zl, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_zl);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_zl,n_zl);

  //transform sigma points into measurement space
  for(int k=0; k<(2 * n_aug_ + 1); k++){
    Zsig(0,k) = Xsig_pred_(0,k);
    Zsig(1,k) = Xsig_pred_(1,k);
  }
  //calculate mean predicted measurement
  z_pred = Zsig*weights_;

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for(int k=0; k<(2 * n_aug_ + 1); k++){
      //state difference
      VectorXd z_diff = (Zsig.col(k) - z_pred);
      S = S + weights_(k)* z_diff * z_diff.transpose();
  }
  S = S + L_;

///*  Update State

  //create vector for incoming lidar measurement
  VectorXd z = VectorXd(n_zl);
  z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc_L = MatrixXd(n_x_, n_zl);

  //calculate cross correlation matrix
  Tc_L.fill(0.0);
  for(int k=0; k<(2 * n_aug_ + 1); k++){
      //state difference
      VectorXd x_diff = (Xsig_pred_.col(k) - x_);
      // angle normalization
      NormalizeAngle(x_diff(3));

      //measurement difference
      VectorXd z_diff = (Zsig.col(k) - z_pred);

      Tc_L = Tc_L + weights_(k)* x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
   MatrixXd K_ = MatrixXd(n_x_,n_zl);
   K_ = Tc_L*S.inverse();

  //update state mean and covariance matrix
  VectorXd z_diff_2 = (z-z_pred);

  x_ = x_ + K_*z_diff_2;
  P_ = P_ - K_*S*K_.transpose();

  ///calculate the lidar NIS.

  NIS_laser_ = z_diff_2.transpose()*S.inverse()*z_diff_2;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) { //MeasurementPackage meas_package const VectorXd &z
  /**
  Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  */

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_zr, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_zr);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_zr,n_zr);

  //transform sigma points into measurement space
  for(int k=0; k<(2 * n_aug_ + 1); k++){
    double dRho_ = sqrt(Xsig_pred_(0,k)*Xsig_pred_(0,k) + Xsig_pred_(1,k)*Xsig_pred_(1,k));
    if(dRho_ < 0.001){
      dRho_ = eps;
    }
    double dPhi_;

    if(abs(Xsig_pred_(0,k))<eps and abs(Xsig_pred_(1,k))<eps){
      dPhi_ = 0.0;
    }
    else{
       dPhi_ = atan2(Xsig_pred_(1,k),Xsig_pred_(0,k));
    }

    double dRhod_ = (Xsig_pred_(0,k)*cos(Xsig_pred_(3,k)) + Xsig_pred_(1,k)*sin(Xsig_pred_(3,k)))*Xsig_pred_(2,k)/dRho_;
    Zsig(0,k) = dRho_;
    Zsig(1,k) = dPhi_;
    Zsig(2,k) = dRhod_;
  }

  //calculate mean predicted measurement
  z_pred = Zsig*weights_;
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for(int k=0; k<(2 * n_aug_ + 1); k++){
      //state difference
      VectorXd z_diff = (Zsig.col(k) - z_pred);

      // angle normalization
      NormalizeAngle(z_diff(1));

      S = S + weights_(k)* z_diff * z_diff.transpose();
  }
  S = S + R_;

///*  Update State

  //create vector for incoming radar measurement
  VectorXd z = VectorXd(n_zr);
  z << meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc_R = MatrixXd(n_x_, n_zr);

  //calculate cross correlation matrix
  Tc_R.fill(0.0);


  for(int k=0; k<(2 * n_aug_ + 1); k++){
      //state difference
      VectorXd x_diff = (Xsig_pred_.col(k) - x_);
      // angle normalization
      NormalizeAngle(x_diff(3));

      //measurement difference
      VectorXd z_diff = (Zsig.col(k) - z_pred);
      // angle normalization
      NormalizeAngle(z_diff(1));

      Tc_R = Tc_R + weights_(k)* x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K_ = MatrixXd(n_x_,n_zr);
  K_ = Tc_R*S.inverse();

  //update state mean and covariance matrix
  VectorXd z_diff_2 = (z-z_pred);
  // angle normalization
  NormalizeAngle(z_diff_2(1));


  x_ = x_ + K_*z_diff_2;
/*  if(x_(2)<0.0){
    x_(3) -=0.00001;//M_PI;
  }*/
  P_ = P_ - K_*S*K_.transpose();


  ///calculate the radar NIS.

  NIS_radar_ = z_diff_2.transpose()*S.inverse()*z_diff_2;
}

void UKF::NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}
