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
        