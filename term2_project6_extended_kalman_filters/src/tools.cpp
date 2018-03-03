#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	VectorXd vDiff;

	//  * the estimation vector size should not be zero
	if(estimations.size() == 0 or estimations.size() != ground_truth.size()) {
 	//  * the estimation vector size should equal ground truth vector size
        cout<<"Invalid estimation or ground_truth data!" << endl;
        return rmse;
	}
    //accumulate squared residuals
    for(size_t i=0; i < estimations.size(); i++){

        vDiff = estimations[i] - ground_truth[i];
        vDiff = vDiff.array() * vDiff.array();
        rmse +=vDiff;
    }
    rmse = rmse/estimations.size();
    rmse = rmse.array().sqrt();
 	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  double brd = 0.0001;
	MatrixXd Hj(3,4);

	Hj<<0,0,0,0,
        0,0,0,0,
        0,0,0,0;
  //recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

  double denom1=sqrt(pow(px,2.)+pow(py,2.));
  double denom2=(pow(px,2.)+pow(py,2.));
  double denom3=pow((pow(px,2.)+pow(py,2.)),3./2.);
	//check division by zero
  if(denom1<brd or denom2<brd or denom3 <brd){
     Hj<<10000, 10000, 0, 0,
         10000, 10000, 0, 0,
         10000, 10000, 10000, 10000;
  }
  else {
      //compute the Jacobian matrix
      Hj(0,0)=px/denom1;
      Hj(0,1)=py/denom1;
      Hj(1,0)=-py/denom2;
      Hj(1,1)=px/denom2;
      Hj(2,0)=py*(vx*py-vy*px)/denom3;
      Hj(2,1)=px*(vy*px-vx*py)/denom3;
      Hj(2,2)=px/denom1;
      Hj(2,3)=py/denom1;
  }
	return Hj;
}
