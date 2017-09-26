#include "PID.h"
#include <cmath>
#include <iostream>

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
    best_error = std::numeric_limits<double>::max(); // set a very large value as initial best error
    
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
    
    dp_sum = VectorSum(dp);
    best_error = TotalError(cte);
    
    if (twiddle) {
        if (dp_sum > 1e-6) {
            cout << "Iteration : " << step << endl;
            cout << "Error Sum : " << dp_sum << endl;
            
            // loop for all PID gains
            for (int i = 0; i < 3; i++){
                
            }
        }
    }
    
    step++;
}

double PID::TotalError(double cte) {
    
    double err;
    err = pow(cte, 2);
    
    return err;
}

double VectorSum(std::vector<double> vec){
    
    double vec_sum = 0.0;
    for (int i=0; i<vec.size(); i++){
        vec_sum += vec[i];
    }
    
    return vec_sum;
}

void PID::UpdateGain(int index, double value) {
    
    // Update Kp
    
    // Update Ki
    
    // Update Kd
}
