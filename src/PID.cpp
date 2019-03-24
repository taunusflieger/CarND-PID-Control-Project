#include "PID.h"
#include <limits>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte)
{
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::Output()
{
  // Limit between zero and minus one
  return std::max(std::min(1.0, -(Kp * p_error + Kd * d_error + Ki * i_error)), -1.0);
  ;
}
