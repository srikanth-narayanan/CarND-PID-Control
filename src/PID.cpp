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
    
    
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

