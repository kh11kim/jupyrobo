import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import random
import numpy.linalg as LA

class Node():
    def __init__(self, position, parent=None):
        self.position = np.array(position)
        self.parent = parent
        self.cost = 0 # for rrt*

    def __sub__(self, other):
        return LA.norm(self.position - other.position)
    
    def __repr__(self):
        return 'pos:{0}'.format(self.position)
    
    def show(self, **kwargs):
        plt.gca().scatter(self.position[0], self.position[1], **kwargs)
        plt.xlim([0,10])
        plt.ylim([0,10])

class RRTAlg():
    def __init__(self, start=None, end=None, new_node_dist=0.5, smpl_area=[0,10,0,10], obstacles=None):
        self.start = start
        self.end = end
        self.new_node_dist = new_node_dist
        self.smpl_area = smpl_area # [xmin, xmax, ymin, ymax]
        self.obstacles = obstacles
        self.tree = [self.start]
    
    def get_rnd_point(self):
        rnd_pos = np.array([random.uniform(*self.smpl_area[0:2]),
                            random.uniform(*self.smpl_area[2:])])
        return Node(rnd_pos)

    def is_goal(self, node, margin=0.5):
        if node-self.end < margin:
            return True
        return False
    
    def is_collision(self, node, margin):
        for obs in self.obstacles:
            obs_pos = np.array(obs[:2])
            obs_radius = obs[2]
            distance = LA.norm(node.position - obs_pos) - obs_radius - margin
            if distance < 0:
                return True
        return False
    
    def is_in_area(self, node):
        xpos, ypos = node.position
        is_in_xarea = (self.smpl_area[0] < xpos) & (xpos < self.smpl_area[1])
        is_in_yarea = (self.smpl_area[2] < ypos) & (ypos < self.smpl_area[3])
        return is_in_xarea & is_in_yarea
    
    def find_nearest_node(self, rnd_node):
        distances = [rnd_node-node for node in self.tree]
        minidx = distances.index(min(distances))
        return self.tree[minidx]
    
class RRT(RRTAlg):
    def show(self):
        self.plot_tree()
        self.plot_obstacles()
        self.start.show(color='r', label='start')
        self.end.show(color='b', label='goal')
        self.plot_final_path()
        
    def plot_tree(self):
        ax = plt.gca()
        for node in self.tree:
            if node.parent:
                pts = np.array([node.position, node.parent.position])
                ax.plot(*pts.T, color='k')
        return ax
    
    def plot_obstacles(self):
        ax = plt.gca()
        for obs in self.obstacles:
            c = mpatches.Circle(obs[:2], obs[2], fc='k')
            im = ax.add_artist(c)
        return im
    
    def plot_final_path(self):
        if not 'goal_node' in dir(self):
            return False
        
        ax = plt.gca()
        temp_node = self.goal_node
        while temp_node.parent:
            pts = np.array([temp_node.position, temp_node.parent.position])
            im = ax.plot(*pts.T, color='r')
            temp_node = temp_node.parent
        return im

class RRTstarAlg(RRTAlg):

    def find_neighbor_nodes(self, new_node, dist=1):
        neighbor_list = []
        for i,node in enumerate(self.tree):
            if (new_node - node) < dist:
                neighbor_list.append((i, node))
        return neighbor_list

class RRTstar(RRTstarAlg):
    
    def show(self):
        self.plot_tree()
        self.plot_obstacles()
        self.start.show(color='r', label='start')
        self.end.show(color='b', label='goal')
        self.plot_final_path()
        
    def plot_tree(self):
        ax = plt.gca()
        for node in self.tree:
            if node.parent:
                pts = np.array([node.position, node.parent.position])
                ax.plot(*pts.T, color='k')
        return ax
    
    def plot_obstacles(self):
        ax = plt.gca()
        for obs in self.obstacles:
            c = mpatches.Circle(obs[:2], obs[2], fc='k')
            im = ax.add_artist(c)
        return im
    
    def plot_final_path(self):
        if not 'goal_node' in dir(self):
            return False
        
        ax = plt.gca()
        temp_node = self.goal_node
        while temp_node.parent:
            pts = np.array([temp_node.position, temp_node.parent.position])
            im = ax.plot(*pts.T, color='r')
            temp_node = temp_node.parent
        return im
    

    

        