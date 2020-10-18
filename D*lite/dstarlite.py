import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class Node():
    def __init__(self, pos):
        self.pos = pos
        self.g = np.inf
        self.rhs = np.inf
        self.h = np.inf
    
    def __eq__(self,other):
        return self.pos == other.pos
    
    def __sub__(self, other):
        #1-norm, manhattan distance
        return np.linalg.norm(np.array(self.pos) - np.array(other.pos), 1) 
    
    def __lt__(self, other):
        key1 = (min(self.g,self.rhs)+self.h, min(self.g,self.rhs))
        key2 = (min(other.g,other.rhs)+other.h, min(other.g,other.rhs))
        return key1 < key2
    def __repr__(self):
        return '{0},{1},{2},{3}'.format(self.pos,self.g,self.rhs,self.h)

class Graph():
    def __init__(self, maps):
        self.maps = maps
        self.make_node_map()

    def make_node_map(self):
        map_w, map_h = self.maps.shape[1], self.maps.shape[0]
        self.node_map = np.empty([map_w, map_h], dtype=Node)
        for i in range(map_w):
            for j in range(map_h):
                self.node_map[i,j] = Node((i,j))

    def get_distance(self, node1, node2):
        pos1, pos2 = node1.pos, node2.pos
        if self.is_obstacle(pos1) or self.is_obstacle(pos2):
            return np.inf
        manhattan = np.linalg.norm(np.array(pos1)-np.array(pos2),1)
        if manhattan==1:
            return 1
        elif manhattan==2:
            return 1.4
        else:
            return np.inf
    
    def get_neighbor_nodes(self, node, direction):
        x,y = node.pos
        if direction=='hv':
            nbrs = [(x+1,y), (x,y-1), (x-1,y), (x,y+1)]
        elif direction=='diag':
            nbrs = [(x+1,y+1), (x+1,y-1), (x-1,y-1), (x-1,y+1)]
        elif direction=='all':
            nbrs = [(x+1,y), (x,y-1), (x-1,y), (x,y+1), 
                    (x+1,y+1), (x+1,y-1), (x-1,y-1), (x-1,y+1)]
        output = []
        for nbr in nbrs:
            if not (self.is_obstacle(nbr) or self.is_wall(nbr)):
                output.append(self.node_map[nbr])
        return output

    def is_obstacle(self, pos):
        return self.maps[pos]==6

    def is_wall(self, pos):
        return self.maps[pos]==9
    

        
class DstarLiteAlg(Graph):
    def __init__(self, start, end, maps):
        self.maps = maps
        Graph.__init__(self, maps)
        self.start = self.node_map[start]
        self.end = self.node_map[end]
        self.end.rhs = 0.0
        self.open_list = [self.end]

    def update_vertex(self,node):
        if node != self.end :
            nbr_g_list = [nbr.g+self.get_distance(node,nbr) for nbr in self.get_neighbor_nodes(node,'all')]
            node.rhs = np.min(nbr_g_list)
        if node in self.open_list:
            self.open_list.remove(node)
        if node.g != node.rhs:
            self.open_list.append(node)
    
    def compute_shortest_path(self):
        max_iter = 1000
        for i in range(max_iter):
            is_find_goal = (self.start.g==self.start.rhs) and (self.start.g != np.inf)
            is_open_empty = len(self.open_list) == 0
            if is_find_goal or is_open_empty:
                break

            curr = self.open_list.pop(0)
            if curr.g > curr.rhs:
                curr.g = curr.rhs
                for pred in self.get_neighbor_nodes(curr,'all'):
                    self.update_vertex(pred)
            else:
                curr.g = np.inf
                self.update_vertex(curr)
                for pred in self.get_neighbor_nodes(curr,'all'):
                    self.update_vertex(pred)
            self.open_list.sort()

    def get_node_to_go(self, node):
        g_list = {nbr.g:nbr.pos for nbr in self.get_neighbor_nodes(node,'all')}
        min_g_pos = g_list[min(g_list.keys())]
        next_node = self.node_map[min_g_pos]
        return next_node

    
    def is_collision(self, node):
        return self.maps[node.pos] in [6, 4]
    
    def is_no_path(self):
        return self.start.g == np.inf


class DstarLite(DstarLiteAlg):
    def __init__(self, start, end, maps):
        DstarLiteAlg.__init__(self, start, end, maps)
        #calculate heuristics
        for node in np.ravel(self.node_map):
            node.h = node - self.start
    

    
    def print_map(self, istext=False, show_path=False, show_search_nodes=False):
        ax = plt.gca()
        
        map_to_plot = self.maps.copy()
        robot = plt.Circle(self.start.pos, 0.4, ec='k', fc=None)
        
        
        
        ax.add_artist(robot)
        if istext==True:
            yy,xx = self.node_map.shape
            for y in np.arange(1, yy-1)[::-1]:
                for x in np.arange(1, xx-1):
                    node = self.node_map[x,y]
                    g = str(round(node.g, 2))
                    rhs = str(round(node.rhs, 2))
                    ax.text(x,y,'g:'+g+'\n rhs:' +rhs, horizontalalignment='center', verticalalignment='center')
                    
        if show_path==True:
            path = pd.DataFrame(columns=['x','y'])
            curr = self.start
            path.loc[0,:] = curr.pos
            
            for iter_ in range(1000):
                if curr == self.end:
                    break
                idx = len(path)
                curr = self.get_node_to_go(curr)
                path.loc[idx,:] = curr.pos
            plt.plot(path.x,path.y,'o-',color='k')
        
        if show_search_nodes==True:
            for node in np.ravel(self.node_map):
                is_searched = (node.g != np.inf) | (node.rhs != np.inf)
                if is_searched and not self.is_collision(node):
                    map_to_plot[node.pos] = 1
        
        ax.imshow(map_to_plot.T, origin='lower', cmap='inferno_r')
