#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Twiddle Parameters
  */
  bool twiddle;
  int step;
  std::vector<double> dp;
  double best_error;
  double dp_sum;
    
    

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double dt);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double cte);

  /*
  * Calculate the total sum of a vector.
  */
  double VectorSum(std::vector<double> vec);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateGain(int index, double value);
};

#endif /* PID_H */
