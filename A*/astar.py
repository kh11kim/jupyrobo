import numpy as np
import pandas as pd

class Node():
    def __init__(self, position):
        self.position = tuple(position)
        self.f = self.h = 0
        self.g = np.inf
        self.parent = None
        
    def __sub__(self, other):
        # Manhattan distance
        return(sum([abs(first-last) for first, last in zip(self.position, other.position)]))
    
    def __lt__(self, other):
        # Define Less-Then operator for sorting
        return self.f < other.f
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
        return 'position : {0}, f : {1}\n'.format(self.position, self.f)


class Astar():
    open_list = []
    closed_list = []
    
    def __init__(self, start=None, end=None, maze=None):
        self.start = start
        self.end = end
        self.maze = maze
    
    def get_neighbor_node(self, curr_node):
        x, y = curr_node.position
        nbr_positions = [(x+1,y), (x,y-1), (x-1,y), (x,y+1)]
        return [Node(pos) for pos in nbr_positions if self.maze[pos]!=5] # 5 for wall
        
    def is_goal(self, node):
        return node == self.end
    
    def calc_cost(self, node):
        #calc cost
        node.g = node.parent.g + 1            # the cost of the cheapest path from start
        node.h = self.end - node              # heuristic : manhattan distance
        node.f = node.g + node.h    # best guess
    
    def is_update_condition(self, node, tentative_past_cost):
        #if node is in open list and cost is higher, update (delete and add).
        #if node is in open list and cost is lower, pass.
        #if node is not in open list, add
        
        for temp_node in self.open_list:
            if (temp_node == node):
                if (temp_node.g > tentative_past_cost):
                    del(temp_node) #update : delete and add
                    return True
                else:
                    return False
        return True
                
