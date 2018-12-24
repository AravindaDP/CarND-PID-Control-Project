#ifndef TWIDDLINGPID_H
#define TWIDDLINGPID_H

#include<vector>
#include <numeric>
#include "PID.h"

class TwiddlingPID : public PID {
public:
  /*
  * Constructor
  */
  TwiddlingPID(double init_dp[3], double max_cte, int n);

  /*
  * Destructor.
  */
  virtual ~TwiddlingPID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  
  double best_error;

private:
  /*
  * Errors
  */
  double error_;

  /*
  * Counters/Flags
  */
  int i_;
  int tuning_idx_;
  bool increment_;

  /*
  * Parameters
  */
  int n_;
  double dp_[3];
  double max_cte_;

  /*
  * Run Twiddle loop
  */
  void Twiddle();

  /*
  * Helper function to adjust currently tuning coefficient.
  */
  void UpdateCoefficient(double dp);

  /*
  * Selects Next Coefficient for tuning.
  */
  void NextCoefficient() {
    double dp_sum = std::accumulate(dp_, dp_+3, 0.0);
    if (dp_sum < 0.001){
      return;
    }
    do 
    {
      tuning_idx_ = (tuning_idx_ + 1) % 3;
    } while (dp_[tuning_idx_] < 0.0001);
  }
};

#endif /* TWIDDLINGPID_H */
