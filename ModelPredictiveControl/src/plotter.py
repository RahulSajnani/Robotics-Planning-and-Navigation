import numpy as np
import matplotlib.pyplot as plt


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

        plt.plot(x_start[0], x_start[1], "ro")
        plt.plot(x_goal[0], x_goal[1], "go")

        for obs in obstacles:
            self.plotCircle(obs[0], obs[1], obs[2])        


        for i in range(X.shape[0] - 1):            
            plt.plot([X[i], X[i + 1]], [Y[i], Y[i + 1]])
            plt.pause(0.1)  
            plt.axis('equal')
        
        plt.show()