#include "PID.h"
#include <cmath>
#include <iostream>
#include <uWS/uWS.h>
#include <numeric>

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
    best_error = std::numeric_limits<double>::max();; // set a very large value
    param_state = 0; // 0 - for kp, 1 - ki and 2 - kd
    twiddle_step = 1;
    kp_state = ki_state = kd_state = 0; // a state of 0 is default, 1 add, 2 sub
    kp_cycle = 0, ki_cycle = 1, kd_cycle = 1; // one twiddle cycle 0 - continue, 1 - next param
    settle_step = 100;
    eval_step = 600;
    twiddle = true;
}

void PID::UpdateError(double cte) {
    
    // Update error
    if (step == 1){
        // Initial d_error
        p_error = cte;
    }
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    
    if (step % (settle_step + eval_step) > settle_step){
        //cout << "Iteration : " << step << endl;
        total_error += pow(cte, 2);
    }
    
    // Apply twiddle after the simulation steps are reached for the next cycle
    if (twiddle && step % (settle_step + eval_step) == 0) {
        for (auto& n : dp)
            dp_sum += n;
        cout << "Error Sum : " << dp_sum << endl;
        cout << "Total Error : " << total_error << endl;
        cout << "Best Error : " << best_error << endl;
        
        /*
         * KP Iteration
         */
        if (kp_cycle == 0){
            // Do stuff for KP
            cout << "IN KP !" << endl;
            if(kp_state == 0){
                // Do things in default state (1st entry point every cycle)
                Kp += dp[0];
                kp_state = 1; // added
                cout << "In addition " << endl;
            }
            else if(kp_state == 1){
                // Do things if previous state was add
                if (total_error < best_error){
                    best_error = total_error;
                    if (step != (settle_step + eval_step)){
                        // no increase if its the first time
                        dp[0] *= 1.1; // Increase dp by 10%
                    }
                    kp_cycle = 1; // Pause KP
                    kd_cycle = 0; /// Next cycle go to KD
                    kp_state = 0; // reset for next run;
                    cout << "Addition Improved " << endl;
                }
                else{
                    Kp -= 2 * dp[0];
                    kp_state = 2; // Subtracted
                    cout << "In Subtraction" << endl;
                }
            }
            else if(kp_state == 2){
                
                // Do things if previous state was subtract
                if (total_error < best_error){
                    best_error = total_error;
                    dp[0] *= 1.1; // Increase dp by 10%
                    kp_cycle = 1; // Pause KP
                    kd_cycle = 0; /// Next cycle go to KD
                    kp_state = 0; // reset for next run;
                    cout << "Subtration Improved " << endl;
                }
                else{
                    Kp += dp[0]; // bring kp back to original state
                    dp[0] *= 0.9; // Take 90%
                    kp_cycle = 1; // Pause KP
                    kd_cycle = 0; /// Next cycle go to KD
                    kp_state = 0; // reset for next run;
                    cout << "Nothing Improved, Reverting back" << endl;
                }
            }
        }
        
        /*
         * KD Iteration
         */
        else if (kd_cycle == 0){
            // Do Stuff for KD
            cout << "IN KD !" << endl;
            if(kd_state == 0){
                // Do things in default state (1st entry point every cycle)
                Kd += dp[2];
                kd_state = 1; // added
                cout << "In addition " << endl;
            }
            else if(kd_state == 1){
                // Do things if previous state was add
                // Do things if previous state was add
                if (total_error < best_error){
                    best_error = total_error;
                    dp[2] *= 1.1; // Increase dp by 10%
                    kd_cycle = 1; // Pause KD
                    ki_cycle = 0; /// Next cycle go to Ki
                    kd_state = 0; // reset for next run;
                    cout << "Addition Improved " << endl;
                }
                else{
                    Kd -= 2 * dp[2];
                    kd_state = 2; // Subtracted
                    cout << "In Subtraction" << endl;
                }
            }
            else if(kd_state == 2){
                // Do things if previous state was subtract
                if (total_error < best_error){
                    best_error = total_error;
                    dp[2] *= 1.1; // Increase dp by 10%
                    kd_cycle = 1; // Pause KD
                    ki_cycle = 0; /// Next cycle go to Ki
                    kd_state = 0; // reset for next run;
                    cout << "Subtration Improved " << endl;
                }
                else{
                    Kd += dp[2]; // bring kp back to original state
                    dp[2] *= 0.9; // Take 90%
                    kd_cycle = 1; // Pause KD
                    ki_cycle = 0; /// Next cycle go to KI
                    kd_state = 0; // reset for next run;
                    cout << "Nothing Improved, Reverting back" << endl;
                }
            }
        }
        
        /*
         * KI Iteration
         */
        else if (ki_cycle == 0){
            // Do Stuff for KI
            cout << "IN KI !" << endl;
            if(ki_state == 0){
                // Do things in default state (1st entry point every cycle)
                Ki += dp[1];
                ki_state = 1; // added
                cout << "In addition " << endl;
            }
            else if(ki_state == 1){
                // Do things if previous state was add
                if (total_error < best_error){
                    best_error = total_error;
                    dp[1] *= 1.1; // Increase dp by 10%
                    ki_cycle = 1; // Pause KI
                    kp_cycle = 0; /// Next cycle go to Kp
                    ki_state = 0; // reset for next run;
                    cout << "Addition Improved " << endl;
                }
                else{
                    Ki -= 2 * dp[1];
                    ki_state = 2; // Subtracted
                    cout << "In Subtraction" << endl;
                }
            }
            else if(ki_state == 2){
                // Do things if previous state was subtract
                if (total_error < best_error){
                    best_error = total_error;
                    dp[1] *= 1.1; // Increase dp by 10%
                    ki_cycle = 1; // Pause KI
                    kp_cycle = 0; /// Next cycle go to Kp
                    ki_state = 0; // reset for next run;
                    cout << "Subtration Improved " << endl;
                }
                else{
                    Ki += dp[1]; // bring kp back to original state
                    dp[1] *= 0.9; // Take 90%
                    ki_cycle = 1; // Pause KI
                    kp_cycle = 0; /// Next cycle go to Kp
                    ki_state = 0; // reset for next run;
                    cout << "Nothing Improved, Reverting back" << endl;
                }
            }
        }
        
        total_error = 0.0;
        //step = 1;
        cout << "Updated Gain" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
        Restart(ws);
    }
    step++;
    
}

double PID::TotalError(double cte) {
    
    double err;
    err = pow(cte, 2);
    
    return err;
}


void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
