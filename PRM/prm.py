import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy.linalg as LA
import random

class Node():
    def __init__(self, position, edge=[]):
        self.position = np.array(position)
        self.edge = edge
        self.f = np.inf
        self.g = 0
        self.parent = None
        
    def __sub__(self, other):
        return LA.norm(self.position-other.position)
    
    def __repr__(self):
        return 'pos:{0}, f:{1}, edge_num:{2}'.format(self.position, self.f, len(self.edge))
    
    def __eq__(self, other):
        return np.array_equal(self.position, other.position)
    
    def __lt__(self,other):
        return self.f < other.f
    
    def show(self, **kwargs):
        plt.gca().scatter(self.position[0], self.position[1], **kwargs)
        plt.xlim([0,10])
        plt.ylim([0,10])

class PRMAlg():
    nodes = []
    
    def __init__(self, start, end, smpl_area, obstacles):
        self.start = start
        self.end = end
        self.smpl_area = smpl_area
        self.obstacles = obstacles
    
    def get_rnd_smpls(self, num):
        output = []
        area = self.smpl_area
        while len(output) < num:
            pos = np.array((random.uniform(*area[0:2]), random.uniform(*area[2:4])))
            if not self.node_is_collision(pos):
                output.append(pos)
        return np.array(output)
            
    def edge_is_collision(self, pos1, pos2):
        step = (pos2-pos1)/10
        points = [pos1 + step*i for i in np.arange(11)]
        for pt in points:
            for obs in self.obstacles:
                obs_pos = np.array(obs[0:2])
                obs_radius = obs[2]
                if LA.norm(pt - obs_pos) <= obs_radius:
                    return True
        return False
    def node_is_collision(self, pos):
        for obs in self.obstacles:
            obs_pos = np.array(obs[0:2])
            obs_radius = obs[2]
            if LA.norm(pos - obs_pos) <= obs_radius:
                return True
        return False
    
    def node_get_neighbors(self, node, k):
        distances = [(node-node_, idx) for idx, node_ in enumerate(self.nodes)]
        distances.sort()
        return [self.nodes[idx] for (_, idx) in distances[:k]]
    
    def is_goal(self, node, margin=0.5):
        if node-self.end < margin:
            return True
        return False

class AstarAlg():
    open_list = []
    closed_list = []
    
    def __init__(self, start=None, end=None):
        self.start = start
        self.end = end
    
    def get_neighbor_node(self, curr_node):
        return curr_node.edge
        
    def is_goal(self, node):
        return node == self.end
    
    def calc_cost(self, node):
        #calc cost
        if type(node.parent) == type(None):
            node.g = 1
        else:
            node.g = node.parent.g + 1            # the cost of the cheapest path from start
        node.h = self.end - node              # heuristic : manhattan distance
        node.f = node.g + node.h    # best guess
        return node
    
    def is_update_condition(self, node):
        #if node is in open list and cost is higher, update (delete and add).
        #if node is in open list and cost is lower, pass.
        #if node is not in open list, add
        
        for temp_node in self.open_list:
            if (temp_node == node):
                if (temp_node.f > node.f):
                    del(temp_node) #update : delete and add
                    return True
                else:
                    return False
        return True
                
class PRM(PRMAlg, AstarAlg):
    def __init__(self, start, end, smpl_area, obstacles):
        PRMAlg.__init__(self, start, end, smpl_area, obstacles)
        
    def plot_node(self):
        ax = plt.gca()
        nodes_pos = np.array([node.position.tolist() for node in self.nodes])
        plt.scatter(*nodes_pos.T)
        
    def plot_edge(self):
        ax = plt.gca()
        visited = []
        for node in self.nodes:
            visited.append(node)
            for edge in node.edge:
                if not edge in visited:
                    pts = np.array([node.position, edge.position])
                    plt.plot(*pts.T,'k', alpha=0.2)
                    
    def plot_graph(self):
        self.plot_edge()
        self.plot_node()
        
    def plot_lim(self):
        ax = plt.gca()
        ax.set_xlim(self.smpl_area[0:2])
        ax.set_ylim(self.smpl_area[2:4])
    
        
    def plot_obstacles(self):
        ax = plt.gca()
        circles = [mpatches.Circle(obs[:2], obs[2], color='k') for obs in self.obstacles]
        for c in circles:
            ax.add_artist(c)
        self.plot_lim()