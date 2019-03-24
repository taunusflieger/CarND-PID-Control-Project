#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;


  // For twiddle - number of updates received
  int update_cnt;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void Twiddle();

  double Output();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

 


  double best_error;
  double total_error;

  double dp_p, dp_i, dp_d;
  int count_threshold;
  int coeff_idx;
  bool increment;
};

#endif  // PID_H