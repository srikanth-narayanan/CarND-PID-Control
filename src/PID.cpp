#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // Initialise gain values
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    
    p_error = i_error = d_error = 0.0;
    
    // Intialise twiddle param
    step = 1;
    dp = {0.1*PID::Kp, 0.1*PID::Ki, 0.1*PID::Kd}; // Take 10% of the gains as initial
    
}

void PID::UpdateError(double cte, double dt) {
    // Update error
    if (step == 1){
        // Initial d_error
        p_error = cte;
    }
    d_error = (cte - p_error) / dt;
    p_error = cte;
    i_error += cte;
    
    step++;
}

double PID::TotalError() {
}

