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
        self.line_coeff = line_coeff #a, b, c from ax+by+c=0
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

class PurePursuit():
    def __init__(self, line_coeff, L, lookahead_distance):
        self.line_coeff = line_coeff #a, b, c from ax + by + c=0
        self.L = L
        self.ld = lookahead_distance

    def get_nearest_point(self, curr_point):
        normal_vector = np.array(self.line_coeff[:2])/LA.norm(self.line_coeff[:2])
        point_on_line = np.array([-self.line_coeff[2] / self.line_coeff[0], 0]) # y=0
        vector = point_on_line - curr_point
        d = vector @ normal_vector
        return curr_point + d * normal_vector

    def get_lookahead_point(self, curr_point):
        a, b, c = self.line_coeff
        point_on_line = np.array([0, -c/b]) # get point at x=0
        nearest_point = self.get_nearest_point(curr_point)
        dist = LA.norm(curr_point - nearest_point)
        if abs(dist) > self.ld:
            return nearest_point
        else:
            normal_vector_through_line = (nearest_point - point_on_line)/LA.norm(nearest_point - point_on_line)
            #if normal vector is heading left, reverse
            if normal_vector_through_line[0] < 0:
                normal_vector_through_line = -normal_vector_through_line
            return nearest_point + normal_vector_through_line * np.sqrt(self.ld**2 - dist**2)

    def ctrl(self, state):
        lookahead_point = self.get_lookahead_point(state[:2])
        theta = state[2]
        ld_vector = lookahead_point - state[:2]
        alpha = np.arctan2(*ld_vector[::-1]) - theta
        lookahead_distance = LA.norm(np.array(state[:2]) - np.array(lookahead_point))
        return np.arctan2(2*self.L*np.sin(alpha), lookahead_distance)

