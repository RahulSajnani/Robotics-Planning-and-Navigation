from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
import numpy as np

'''
Authors:

Rahul Sajnani
Amarthya Sasi Kiran
Abhiram Kadiyala

Date: 5th February 2021
'''

class Graph:

    def __init__(self):

        self.nodes = None
        self.edges = []
    
    def addEdge(self, node_index_1, node_index_2):
        '''
        Function to add edges in graph 
        '''

        # print(node_index_1, node_index_2)
        self.edges[node_index_1].append(node_index_2)
        self.edges[node_index_2].append(node_index_1)

    def addNode(self, new_node):
        '''
        Function to add Node
        '''
        if self.nodes is None:
            self.nodes = np.array([new_node])
        else:
            self.nodes = np.vstack((self.nodes, new_node))
        self.edges.append([])
        node_index = len(self.nodes) - 1
        
        return node_index
    
    def findPathToGoal(self, parent_node_index, goal_node_index):
        '''
        Find the path to goal
        '''
        
        self.traversed_nodes = np.zeros(self.nodes.shape[0])
        path = self.depthFirstSearch(parent_node_index, goal_node_index)
        edge_list = []
        
        index = len(path) - 1
        while index > 1:

            edge_list.append((path[index], path[index - 1]))
            index = index - 1
        
        return self.nodes[np.array(path)], edge_list

        
    def depthFirstSearch(self, parent_node_index, goal_node_index):
        '''
        Depth first search get path
        '''
        
        self.traversed_nodes[parent_node_index] = 1
        
        if parent_node_index == goal_node_index:
            return [parent_node_index]

        for next_node_index in self.edges[parent_node_index]:
            
            if not self.traversed_nodes[next_node_index]:
                # if node is not traversed

                path_list = self.depthFirstSearch(next_node_index, goal_node_index)

                # Path is null if goal is not found from this point
                if path_list is not None:
                    # Add current node to the path as it lies in the path to goal
                    path_list.append(parent_node_index)
                    return path_list                
            
        return None
        