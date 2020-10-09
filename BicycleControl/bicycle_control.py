import numpy as np
import pandas as pd
import numpy.linalg as LA
from bicycle import *

class BicycleControl():
    
    def __init__(self, gamma_max=np.pi/4):
        self.gamma_max = gamma_max

    def move_to_point(self, state, ref, v_gain=1, gamma_gain=1):
        convert_angle = lambda x: x if (-np.pi < x)&(x <= np.pi) \
                                    else x - np.sign(x)*np.pi*2

        xy_d = ref[:2]
        xy_err = (xy_d - state[:2])  
        
        theta_d = np.arctan2(xy_err[1], xy_err[0])     
        theta_err = theta_d - state[2]
        theta_err = convert_angle(theta_err)

        v = LA.norm(xy_err) * v_gain
        gamma = theta_err * gamma_gain

        if abs(gamma) > self.gamma_max:
            gamma = np.sign(gamma)* self.gamma_max
        return v, gamma
    
class Stanley():
    def __init__(self, line_coeff, gain, gamma_max):
        self.line_coeff = line_coeff
        self.theta_d = np.arctan2(-line_coeff[0], line_coeff[1])
        self.gain = gain
        self.gamma_max = gamma_max

    def ctrl(self, state, v):
        convert_angle = lambda x: x if (-np.pi < x)&(x <= np.pi) \
                                    else x - np.sign(x)*np.pi*2
        
        theta = state[2]
        heading_err = convert_angle(self.theta_d - theta)
        crosstrack_err = self.get_crosstrack_error(state)
        gamma = heading_err + np.arctan2(self.gain * crosstrack_err, v)
        if abs(gamma) > self.gamma_max: gamma = np.sign(gamma)*self.gamma_max
        
        return gamma

    def get_crosstrack_error(self, state):
        a, b, c = self.line_coeff
        x, y = state[:2]
        return -(a*x+b*y+c)/np.sqrt(LA.norm([a, b]))

