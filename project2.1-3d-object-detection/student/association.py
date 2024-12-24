# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        # Association
        # - replace association_matrix with the actual association matrix
        #   based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        
        track_length = len(track_list)
        meas_length = len(meas_list)
        self.association_matrix = np.inf * np.ones((track_length, meas_length))
        self.unassigned_tracks = list(range(track_length))
        self.unassigned_meas = list(range(meas_length))
        
        # Calculate association matrix
        for i, track in enumerate(track_list):
            for j, meas in enumerate(meas_list):
                if self.gating(self.MHD(track, meas, KF), meas.sensor):
                    self.association_matrix[i, j] = self.MHD(track, meas, KF)
                
    def get_closest_track_and_meas(self):
        
        # Find closest track and measurement
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
            
        association_matrix = self.association_matrix

        # Find minimum entry in association matrix
        min_ind = np.unravel_index(
            np.argmin(association_matrix), 
            association_matrix.shape
        )
        track_ind = min_ind[0]
        meas_ind = min_ind[1]

        # Prepare for next iteration
        association_matrix = np.delete(
            association_matrix, 
            track_ind, 
            axis = 0
        )
        association_matrix = np.delete(
            association_matrix,
            meas_ind,
            axis = 1
        )
        self.association_matrix = association_matrix

        # Update track and measurement
        track_updated = self.unassigned_tracks[track_ind]
        meas_updated = self.unassigned_meas[meas_ind]

        # Remove track and measurement from list
        self.unassigned_tracks.remove(track_updated)
        self.unassigned_meas.remove(meas_updated)

        return track_updated, meas_updated

    def gating(self, MHD, sensor): 

        # Return True if measurement lies inside gate, otherwise False
        
        gate_threshold = chi2.ppf(
            params.gating_threshold,
            sensor.dim_meas
        )

        return (MHD < gate_threshold)
        
    def MHD(self, track, meas, KF):
        
        # Calculate and return Mahalanobis distance
        
        sensor = meas.sensor
        H = sensor.get_H(track.x)
        gamma = KF.gamma(track, meas)
        S = H * track.P * H.T + meas.R
        MHD = gamma.T * S.I * gamma

        return MHD
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)