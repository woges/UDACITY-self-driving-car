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
    Calculate the RMSE here.
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
