#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state(sec7)
   */
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations(sec7,8)
   */
   VectorXd y = z - H_*x_;
   MatrixXd S = H_*P_*H_.transpose() + R_;
   MatrixXd K = P_*H_.transpose()*S.inverse();
   // new state
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   x_ = x_ + K*y;
   P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);
   VectorXd hx(3);
  
   std::cout << "x_" << std::endl;
   std::cout << x_ << std::endl;
   std::cout << "P_" << std::endl;
   std::cout << P_ << std::endl;
   std::cout << "F" << std::endl;
   std::cout << F_ << std::endl;
   std::cout << "H" << std::endl;
   std::cout << H_ << std::endl;
   std::cout << "R" << std::endl;
   std::cout << R_ << std::endl;
   std::cout << "Q" << std::endl;
   std::cout << Q_ << std::endl;

   hx << sqrt(px*px + py*py),
         atan2(py, px),
         (px*vx + py*vy)/sqrt(px*px + py*py);

   //std::cout << hx << std::endl;
   VectorXd y = z - hx;
   std::cout << y << std::endl;
   MatrixXd S = H_*P_*H_.transpose() + R_;
   std::cout << "S" << std::endl;
   std::cout << S << std::endl;
   MatrixXd K = P_*H_.transpose()*S.inverse();
   std::cout << "K" << std::endl;
   std::cout << K << std::endl;
   // new state
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   std::cout << "x_" << std::endl;
   x_ = x_ + K*y;
   std::cout << x_ << std::endl;
   P_ = (I-K*H_)*P_;
   std::cout << P_ << std::endl;
}
