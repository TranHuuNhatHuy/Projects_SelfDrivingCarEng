/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   
   // ================================================================== Initialize PID coefficients

   this->Kp = Kpi;                           // Proportional coefficient
   this->Ki = Kii;                           // Integral coefficient
   this->Kd = Kdi;                           // Derivative coefficient
   this->output_lim_max = output_lim_maxi;   // Max output value
   this->output_lim_min = output_lim_mini;   // Min output value

   cte0 = 0;                                 // Error
   I = 0;                                    // Integral error
   
}


void PID::UpdateError(double cte) {

   // ================================================================== Update PID errors based on cte.

   if(abs(dt) < 0.000001) {
      return;
   }

   double P = Kp * cte;                         // Proportional error
   I += Ki * cte * dt;                          // Integral error

   double D = Kd * (cte - cte0) / dt;           // Derivative error
   action = P + I + D;                          // Total error
   cte0 = cte;                                  // Update previous cte error

}

double PID::TotalError() {
   // ================================================================== Calculate and return the total error
   
   double control = action;                     // Control value

   if(control > output_lim_max) {               // Limit control value to [output_lim_min, output_lim_max]
      control = output_lim_max;
   } else if(control < output_lim_min) {
      control = output_lim_min;
   }

   return control;

}

double PID::UpdateDeltaTime(double new_dt) {
   
   // ================================================================== Update the delta time with new value
   
   dt = new_dt;

   return dt;

}