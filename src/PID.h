#ifndef PID_H
#define PID_H

/**
 * PID Controller.
 */
class PID {
public:
  /*
   * Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;
  
  /*
   * Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;
  
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
   *
   * @param Kp Proportional coefficient.
   * @param Ki Integral coefficient.
   * @param Kd Differential coefficient.
   */
  void Init(double Kp, double Ki, double Kd);
  
  /**
   * Update the PID error variables given cross track error.
   *
   * @param cte Actual cross track error.
   * @param dt  Delta time in seconds.
   */
  void UpdateError(double cte, double dt = 1.0);
  
  /**
   * Returns the actual control value.
   *
   * @return Returns the controlled valued
   */
  double GetControlValue();
  
  /**
   * Calculate the total PID error.
   */
  double TotalError();
};

#endif /* PID_H */
