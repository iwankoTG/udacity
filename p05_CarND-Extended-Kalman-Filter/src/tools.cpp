#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.(sec24)
   */

   VectorXd rmse(4);
   rmse << 0,0,0,0;

   if(estimations.size() == 0){
     cout << "estimation vector is empty" << endl;
     return rmse;
   }
   if(estimations.size() != ground_truth.size()){
     cout << "estimation and ground_truth have different vector sizes" << endl;
     return rmse;
   }

   for (unsigned int i=0; i < estimations.size(); ++i) {
     VectorXd sub = estimations[i] - ground_truth[i];
     sub = sub.array()*sub.array();
     rmse += sub;
   }

   rmse /= estimations.size();
   rmse = rmse.array().sqrt();
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.(sec19,20)
   */
   MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float norm = sqrt(px*px + py*py);
  float norm2 = norm*norm;
  float norm3 = norm*norm*norm;

  // check division by zero
  if(fabs(norm2) < 0.0001){
      cout << "Hj cannot be calculated" << std::endl;
  }else{
  // compute the Jacobian matrix
      Hj << px/norm, py/norm, 0, 0,
           -py/norm2, px/norm2, 0, 0,
            py*(vx*py-vy*px)/norm3, px*(vy*px-vx*py)/norm3, px/norm, py/norm;
  }
  return Hj;
}
