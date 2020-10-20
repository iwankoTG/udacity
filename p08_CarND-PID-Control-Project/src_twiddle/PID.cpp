#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

//void PID::Init(double Kp_, double Ki_, double Kd_) {
void PID::Init(std::vector<double> &p_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   //Kp = Kp_;
   //Ki = Ki_;
   //Kd = Kd_;
   Kp = p_[0];
   Ki = p_[1];
   Kd = p_[2];

   p_error = 0.0;
   i_error = 0.0;
   d_error = 0.0;
   total_error = 0.0;
   total_count = 0;
}

int PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   d_error = cte - p_error;
   p_error = cte;
   i_error += cte;
  
   total_error += cte*cte;
   total_count += 1;
  
   return total_count;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
   return total_error/total_count;
  //return 0.0;  // TODO: Add your total error calc here!
}

double PID::CalcSteering() {
  double steering = -Kp*p_error - Ki*i_error - Kd*d_error;

  if(steering > 1.0){
    steering = 1.0;
  }else if (steering < -1.0){
    steering = -1.0;
  }
  return steering;
}
