import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import disk, dilation, erosion
from utils import *

'''
Authors:

Rahul Sajnani
Amarthya Sasi Kiran
Abhiram Kadiyala

Date: 5th February 2021
'''


class Map:
    '''
    Map class to build and create the game map
    '''
    def __init__(self):

        self.map = None
        self.cmap = None
        self.map_dimensions = None


    def initializeMap(self, map):

        self.map = map
        self.cmap = map
        self.map_dimensions = map.shape

    def checkLocationInMap(self, position):
        '''
        Checks if the given point lies within the map boundaries
        '''
        
        if  (position[0] > 0) and \
            (position[0] < self.map_dimensions[0]) and \
            (position[1] > 0) and \
            (position[1] < self.map_dimensions[1]):
            
            return True
        
        return False

    def createRectangle(self, rect_position, rect_size = (20, 20)):
        '''
        rect_position - top left position of rectangle
        rect_size     - size of the rectangular obstacle
        '''
        end_h = self.map_dimensions[0]
        end_w = self.map_dimensions[1]

        if self.checkLocationInMap(rect_position):
        
            if rect_position[0] + rect_size[0] < self.map_dimensions[0]:
                end_h = rect_position[0] + rect_size[0]
        
            if rect_position[1] + rect_size[1] < self.map_dimensions[1]:
                end_w = rect_position[1] + rect_size[1]

            self.map[rect_position[0]: end_h, rect_position[1]: end_w] = 0
               
    def createConfigMap(self, radius_bot = 10):
        '''
        Create Config map for the input map.
        '''

        map = self.map.copy()
        config_map = (1 - map)
        config_map = dilation(config_map, disk(radius_bot))
        config_map = (1 - config_map)
        
        self.cmap = config_map

    def createCircle(self, circle_top, radius = 20):
        '''
        Create circular object given top left corner of circle
        '''

        disk_matrix = disk(radius)
        h, w = disk_matrix.shape

        if self.checkLocationInMap(circle_top):
            
            end_h = circle_top[0] + h
            end_w = circle_top[1] + w

            if not (end_h < self.map_dimensions[0]):
                end_h = self.map_dimensions[0]
            
            if not (end_w < self.map_dimensions[1]):
                end_w = self.map_dimensions[1]
            
           
            self.map[circle_top[0]: end_h - 1, circle_top[1]: end_w - 1] *= \
            ((disk_matrix == 0)*1)[:end_h - circle_top[0] - 1, :end_w - circle_top[1] - 1]

    def buildMap(self, map_size = (500, 500), num_obstacles = 40, min_obstacle_size = 10, max_obstacle_size = 50):
        
        self.map_dimensions = map_size
        self.map = np.ones(map_size)
        
        # Create map boundaries
        self.map[0, :] = 0
        self.map[:, 0] = 0
        self.map[map_size[0] - 1, :] = 0
        self.map[:, map_size[1] - 1] = 0
        self.cmap = np.copy(self.map)
        
        for i in range(num_obstacles):
            
            position = (np.random.randint(self.map_dimensions[0]),
                        np.random.randint(self.map_dimensions[1]))

            if np.random.rand() < 0.5:
                rect_size = (np.random.randint(min_obstacle_size, max_obstacle_size),
                         np.random.randint(min_obstacle_size, max_obstacle_size))
            
                self.createRectangle(position, rect_size)
            else:
                self.createCircle(position, np.random.randint(min_obstacle_size // 2, max_obstacle_size // 2))

          
if __name__ == "__main__":

    map_object = Map()
    # map_object.initializeMap()
    map_object.buildMap(num_obstacles=70)
    map_object.createConfigMap(radius_bot=3)
    
    
    # check = (map_object.cmap[map_object.map == 0] = 1).copy()
    # print(map_object.cmap.shape)
    check = map_object.cmap.copy()
    check[map_object.map == 0] = 1
    # print(check.shape)
    # plot_images([map_object.map, map_object.cmap, check], (2,2), cmap="gray")
    plot_images([map_object.cmap], (1,1), cmap="gray")
    
    # option = int(input("save map?"))   
    # if option:
    #     plt.imsave("map2.png", map_object.map, cmap="gray")