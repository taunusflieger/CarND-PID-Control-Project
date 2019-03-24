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

  // Init errors
  total_error = 0.0;
  best_error = std::numeric_limits<double>::max();

  count_threshold = 0;
  update_cnt = 0;

  dp_p = Kp / 10.0;
  dp_i = Ki / 10.0;
  dp_d = Kd / 10.0;

  coeff_idx = 0;
  increment = true;
}

void PID::UpdateError(double cte)
{
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if (update_cnt > count_threshold)
    total_error += cte * cte;
  update_cnt++;
}

double PID::TotalError()
{

  return total_error / (update_cnt - count_threshold);
}

double PID::Output()
{
  double output_val = -1.0 * (Kp * p_error + Kd * d_error + Ki * i_error);

  // Limit between zero and minus one
  if (output_val > 1.0)
    output_val = 1.0;
  else if (output_val < -1.0)
    output_val = -1.0;

  return output_val;
}

void PID::Twiddle()
{
  //get Current error
  double current_error = TotalError();

  // reset counters as new segment starts
  total_error = 0;
  update_cnt = 0;

  // If current best error is too large
  if (best_error > 999)
  {
    best_error = current_error;
    Kp += dp_p;
    return;
  }

  // Reinitialize individual errors
  i_error = 0;
  d_error = 0;
  p_error = 0;

  // If the current error is less than best error
  if (current_error < best_error && update_cnt > 2 * count_threshold)
  {
    // record new best error
    best_error = current_error;

    switch (coeff_idx)
    {
    case 0:
      dp_p *= 1.1;
      break;
    case 1:
      dp_i *= 1.1;
      break;
    default:
      dp_d *= 1.1;
    }
    increment = true;
  }
  else
  {
    if (!increment)
    {
      switch (coeff_idx)
      {
      case 0:
        Kp += dp_p;
        dp_p *= 0.9;
        break;
      case 1:
        Ki += dp_i;
        dp_i *= 0.9;
        break;
      default:
        Kd += dp_d;
        dp_d *= 0.9;
      }
      increment = true;
    }
    else
    {
      increment = false;
    }
  }

  // Cyclically adjust coefficient (0,1,2)
  coeff_idx = (coeff_idx + 1) % 3;

  // In any case, Readjust Coefficients after conditional check based on coefficient choice
  switch (coeff_idx)
  {
  case 0:
    if (increment)
      Kp += dp_p;
    else
      Kp -= 2 * dp_p;
    break;
  case 1:
    if (increment)
      Ki += dp_i;
    else
      Ki -= 2 * dp_i;
    break;
  default:
    if (increment)
      Kd += dp_d;
    else
      Kd -= 2 * dp_d;
  }
}
