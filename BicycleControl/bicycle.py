import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class BicycleModel():
    def __init__(self, L, dt, model_type):
        self.L = L
        self.dt = dt
        self.model_type = model_type 

    def model(self, *args):
        if self.model_type == 'rear':
            return self.bicycle_rear_axle(*args)
        if self.model_type == 'front':
            return self.bicycle_front_axle(*args)

    def bicycle_rear_axle(self, state, v, gamma):
        #state : x, y, theta
        x, y, theta = state
        delta = np.array([v*np.cos(theta), v*np.sin(theta), v/self.L*np.tan(gamma)])
        next_state = state + delta  * self.dt
        return next_state
    
    def bicycle_front_axle(self, state, v, gamma):
        #state : x, y, theta
        x, y, theta = state
        delta = np.array([v*np.cos(theta+gamma), 
                          v*np.sin(theta+gamma), 
                          v/self.L*np.sin(gamma)])
        next_state = state + delta  * self.dt
        return next_state

    def set_state(self, state, gamma=0):
        self.state = state
        self.gamma = gamma

class Transformation():
    # Transformation for drawing two wheels
    def convert_se2(self, p, theta):
        p = np.array(p)[:,None]
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c,-s],[s,c]])
        return np.block([[R, p],[np.zeros((1,2)), 1]])

    def invert_se2(self, T):
        theta = np.arctan2(T[1,0], T[0,0])
        p = T[:2, -1]
        return p, theta

class Bicycle(Transformation, BicycleModel):
    wheel_len = 1
    wheel_width = 0.5
    arrow_len = 1
    view = [-10, 10, -10, 10]

    def show(self):
        self.draw_wheels()
        self.draw_arrow()
        plt.axis(self.view)
        plt.grid()
    
    def update(self):
        ax = plt.gca()
        xy, theta = self.calc_wheel_pos(wheel_type='main')
        self.main_wheel.set_xy(xy)
        self.main_wheel.angle = np.rad2deg(theta)
        xy, theta = self.calc_wheel_pos(wheel_type='sub')
        self.sub_wheel.set_xy(xy)
        self.sub_wheel.angle = np.rad2deg(theta)
        self.arrow.set_visible(False)
        self.arrow.remove()
        self.draw_arrow()

    def calc_wheel_pos(self, wheel_type='main'):
        T_s_wheel = self.convert_se2(self.state[:2], self.state[2])
        if wheel_type=='sub':
            if self.model_type == 'rear':
                T_main_sub = self.convert_se2([self.L,0], 0)
            if self.model_type == 'front':
                T_main_sub = self.convert_se2([-self.L,0], 0)
            T_s_wheel = T_s_wheel @ T_main_sub

        is_front_wheel = (self.model_type=='front') & (wheel_type=='main') | \
                         (self.model_type=='rear') & (wheel_type=='sub')
        if is_front_wheel:
            T_steer = self.convert_se2([0,0], self.gamma)
            T_s_wheel = T_s_wheel @ T_steer
        
        draw_pos = - np.array([self.wheel_len, self.wheel_width])/2
        T_wheel_draw = self.convert_se2(draw_pos[:2], 0)
        T_s_draw = T_s_wheel @ T_wheel_draw
        draw_xy, draw_theta = self.invert_se2(T_s_draw)
        return draw_xy, draw_theta
        
    def draw_wheels(self):
        ax = plt.gca()

        xy_main, theta_main = self.calc_wheel_pos(wheel_type='main')
        self.main_wheel = patches.Rectangle(xy_main, self.wheel_len, self.wheel_width, 
                                            angle=np.rad2deg(theta_main),
                                            fc=None, ec='k', alpha=0.5)

        xy_sub, theta_sub = self.calc_wheel_pos(wheel_type='sub')
        self.sub_wheel = patches.Rectangle(xy_sub, self.wheel_len, self.wheel_width, 
                                           angle=np.rad2deg(theta_sub),
                                           fc=None, ec='k', alpha=0.5)
        
        ax.add_patch(self.main_wheel)
        ax.add_patch(self.sub_wheel)
        

    def draw_arrow(self):
        ax = plt.gca()
        x, y, theta = self.state
        if self.model_type == 'front':
            theta = theta + self.gamma
        dx, dy = self.arrow_len*np.cos(theta), self.arrow_len*np.sin(theta)
        self.arrow = plt.arrow(x, y, dx, dy, head_width=0.5, head_length=0.5,
                            length_includes_head=True)
        ax.add_patch(self.arrow)