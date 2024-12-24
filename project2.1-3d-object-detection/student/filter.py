# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        # Implement and return system matrix F
        dt = params.dt
        system_matrix = np.matrix([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        return system_matrix

    def Q(self):
        # Implement and return process noise covariance Q
        q = params.q
        dt = params.dt
        tq3 = (dt**3) / 3 * q
        tq2 = (dt**2) / 2 * q
        tq = dt * q
        process_noise_covariance = np.matrix([
            [tq3, 0, 0, tq2, 0, 0],
            [0, tq3, 0, 0, tq2, 0],
            [0, 0, tq3, 0, 0, tq2],
            [tq2, 0, 0, tq, 0, 0],
            [0, tq2, 0, 0, tq, 0],
            [0, 0, tq2, 0, 0, tq]
        ])

        return process_noise_covariance

    def predict(self, track):
        # Predict state x and estimation error covariance P to next timestep, save x and P in track
        x = self.F() * track.x
        P = self.F() * track.P * self.F().T + self.Q()
        track.set_x(x)
        track.set_P(P)

    def update(self, track, meas):
        # Update state x and covariance P with associated measurement, save x and P in track
        P = track.P
        sensor = meas.sensor
        H = sensor.get_H(track.x)
        S = self.S(track, meas, H)
        K = P * H.T * S.I
        I = np.identity(params.dim_state)
        x = track.x + K * self.gamma(track, meas)
        P = (I - K * H) * track.P

        track.set_x(x)
        track.set_P(P)

        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        # Calculate and return residual gamma
        x = track.x
        z = meas.z
        sensor = meas.sensor
        hx = sensor.get_hx(x)
        res_gamma = z - hx
        
        return res_gamma

    def S(self, track, meas, H):
        # Calculate and return covariance of residual S
        R = meas.R
        P = track.P
        covar_residual = H * P * H.T + R
        
        return covar_residual