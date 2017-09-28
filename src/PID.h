#ifndef PID_H
#define PID_H

#include <vector>
#include <uWS/uWS.h>

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
  int settle_step;
  int eval_step;
  std::vector<double> dp;
  double best_error;
  double dp_sum;
  double total_error;
  int param_state;
  int twiddle_step;
  int kp_state, ki_state, kd_state; // a state of 0 is default, 1 add, 2 sub
  int kp_cycle, ki_cycle, kd_cycle; // one twiddle cycle 0 - continue, 1 - next param
  uWS::WebSocket<uWS::SERVER> ws;
    

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
  * Update the PID error variables given cross track error.
  */
  void UpdateGain(int index, double value);
  
  /*
  * A method to restart the simulator.
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
    
};

#endif /* PID_H */
