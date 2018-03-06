#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <math.h>

using namespace std;
using CONFIG_OUTPUT = tuple<double, double,double, double,double, double,double, double, vector<double>, vector<double>>;
typedef CPPAD_TESTVECTOR(double) Dvector;

// Evaluate a polynomial (using CPPAD).
template<typename scalar_t>
scalar_t cppadPolyeval(Eigen::VectorXd coeffs, scalar_t x, bool bSlope) {
  scalar_t result = 0.0;
  if(!bSlope){
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * CppAD::pow(x, i);
    }
  } else {
    for (int i = 1; i < coeffs.size(); i++) {
      result += i*coeffs[i] * CppAD::pow(x, i-1);
    }
  }
  return result;
}

class MPC {
 public:
  MPC();
  Dvector dvLast_con;

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  void Init() ;

  CONFIG_OUTPUT Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
