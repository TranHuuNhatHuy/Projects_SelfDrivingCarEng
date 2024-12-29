/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

    // ================================================================== Create the PID class

    double cte0;            // Error

    double Kp;              // Proportional coefficient
    double Ki;              // Integral coefficient
    double Kd;              // Derivative coefficient

    double action;
    double output_lim_max;  // Max output limit
    double output_lim_min;  // Min output limit

    double dt;              // Delta time
    double I;               // Integral error

    PID();                  // Constructor

    virtual ~PID();         // Destructor

    // Initialize PID
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    // Update the PID error variables given cross track error.
    void UpdateError(double cte);

    // Calculate the total PID error.
    double TotalError();

    // Update delta time
    double UpdateDeltaTime(double new_dt);
};

#endif //PID_CONTROLLER_H