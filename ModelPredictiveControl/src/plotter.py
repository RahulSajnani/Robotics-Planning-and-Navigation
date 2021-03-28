import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

'''
Author:
    Rahul Sajnani
Date:
    21st March 2021
'''

class Plotter:

    def __init__(self):

        pass
    
    def plotCircle(self, x, y, radius):
        
        circle1 = plt.Circle((x, y), radius, color='black')
        plt.gca().add_patch(circle1)

        
    def plot(self, x_start, x_goal, obstacles, X, Y):
        '''
        Plot the graph
        '''
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        plt.title("Robot path using Model Predictive Control")
        plt.plot(x_start[0], x_start[1], "ro", label = "start position")
        plt.plot(x_goal[0], x_goal[1], "go", label = "goal position")
        plt.legend()

        for obs in obstacles:
            self.plotCircle(obs[0], obs[1], obs[2])        

        red_patch = mpatches.Patch(color='red', label="Initial position")
        green_patch = mpatches.Patch(color='green', label="Goal position")
        
        if len(obstacles) == 0:
            plt.legend(handles=[red_patch, green_patch])
        else:
            black_patch = mpatches.Patch(color='black', label="Obstacles")
            plt.legend(handles=[black_patch, red_patch, green_patch])
        for i in range(X.shape[0] - 1):            
            plt.plot([X[i], X[i + 1]], [Y[i], Y[i + 1]])
            plt.pause(0.1)  
            plt.axis('equal')

        plt.show()