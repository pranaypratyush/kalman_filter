from pykalman import KalmanFilter
from pykalman import UnscentedKalmanFilter
from pykalman import AdditiveUnscentedKalmanFilter
import numpy as np
import matplotlib.pyplot as plt
from math import *
from matrix import *

dt = 0.1



def kalman_estimate_v_a(x,P,measurements) :
    """
    The function "kalman_estimate_v_a" is used to filter the velocity and acceleration
    data using a simple kalman filter.
    The transition matrix (A) = [[1,dt],
                                 [0, 1]]
    The observation matrix (H) = [[1, 0],
                                  [0, 1]]
    # measurement only takes v and a in the form [v,a]
    # measurement input for velocity and acceleration
    """

    kf = KalmanFilter(transition_matrices = [[1, dt], [0, 1]], observation_matrices = [[1, 0], [0, 1]])
    (x,P) = filter_update(x,P,measurements)
    return (x,P)



def kalman_estimate_1D(x,P,va_measurements,x_history):
    """
    This function takes the filtered velocity, acceleration data and covariance matrix (from previous filtration),
    the filtered position history (for only a single dimension) and the measurements (for velocity
    and acceleration). And returns the updated velocity,acceleration data and covariance matrix
    and also updates the position history.
    It uses an Additive Unscented Kalman Filter and calls "kalman_estimate_v_a" in order to achieve this.
    The functions 'lambda x,w' and 'lambda x,v' are respectively the transition and observation functions
    (https://pykalman.github.io/#id3).
    # measurements for 1 dim in the form [v,a] # x_history is history of in format [x1,x2,x3,...]
    """

    (x,P)=kalman_estimate_v_a(x,P,va_measurements)
    aukf = AdditiveUnscentedKalmanFilter(lambda x, w: x + w, lambda x, v: x + v, observation_covariance=1)
    x_history=x_history[-30:] + x[0]*dt + 0.5 * x[1] * dt * dt
    x_filtered,temp = aukf.filter(x_history)

    return (x,P,x_filtered)
