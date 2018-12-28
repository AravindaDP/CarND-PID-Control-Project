#include "TwiddlingPID.h"
#include <iostream>
#include <limits>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

TwiddlingPID::TwiddlingPID(double init_dp[3], double max_cte, int n) {
  std::copy(init_dp, init_dp + 3, dp_);
  max_cte_ = max_cte;
  n_ = n;
  tuning_idx_ = 0;
  increment_ = true;
  error_ = 0;
  i_ = 0;
  best_error = 10 * max_cte*max_cte;
}

TwiddlingPID::~TwiddlingPID() {}

void TwiddlingPID::UpdateError(double cte) {
  PID::UpdateError(cte);
  i_++;
  if (i_ > n_) {
    error_ += cte * cte;
  }
}

double TwiddlingPID::TotalError() {
  double pid_error = PID::TotalError();

  double dp_sum = std::accumulate(dp_, dp_+3, 0.0);
  if (dp_sum < 0.001){ //Twiddling finished. Continue;
    return pid_error;
  }

  if (abs(p_error) > max_cte_ && i_ > 10) { // Crashed
    pid_error = std::numeric_limits<double>::quiet_NaN();
    Twiddle();
  }
  else if (i_ > 2 * n_) { // Success run
    pid_error = std::numeric_limits<double>::quiet_NaN();
    Twiddle();
  }

  return pid_error;
}

void TwiddlingPID::Twiddle() {
  double err = 0;
  if (i_ <= n_) {
    err = max_cte_*max_cte_;
  }
  else {
    err = error_*n_ / (i_*(i_-n_));
  }

  //Re-warped twiddle algorithm
  if (increment_) { 
    if (err < best_error) {
      best_error = err;
      dp_[tuning_idx_] *= 1.1;
      NextCoefficient();
      UpdateCoefficient(dp_[tuning_idx_]);
    }
    else {
      UpdateCoefficient(- dp_[tuning_idx_] * 2);
      increment_ = false;
    }
  }
  else {
    if (err < best_error) {
      best_error = err;
      dp_[tuning_idx_] *= 1.1;
    }
    else {
      UpdateCoefficient(dp_[tuning_idx_]);
      dp_[tuning_idx_] *= 0.9;
    }
    NextCoefficient();
    UpdateCoefficient(dp_[tuning_idx_]);
    increment_ = true;
  }
  error_ = 0;
  i_ = 0;

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void TwiddlingPID::UpdateCoefficient(double dp) {
  switch (tuning_idx_) {
    case 0:
      Kp += dp;
      break;
    case 1:
      Ki += dp;
      break;
    case 2:
      Kd += dp;
      break;
    default:
      break;
  }
}

