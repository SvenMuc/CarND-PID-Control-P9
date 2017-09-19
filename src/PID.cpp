#include "PID.h"
#include <iostream>


/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
}

void PID::UpdateError(double cte, double dt) {
  // The proportional (P) error equals the cross track error
  // The integral (I) error equals the sum of all cross track errors
  // The differential (D) error equals the diff between the actual and the previous cross track error
  d_error_ = (dt >= 0.0000001 ? (cte - p_error_) / dt : 0.0);
  p_error_ = cte;
  i_error_ += cte * dt;
}

double PID::GetControlValue() {
  return -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;
}

double PID::TotalError() {
  return 0.0;
}
