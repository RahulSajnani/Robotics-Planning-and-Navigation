import numpy as np
import matplotlib.pyplot as plt
from agent import *


class VelocityObstacle:

    def __init__(self):
        
        
        pass

    def getVelocityObstacle(self, robot, obstacle):
        '''
        Get velocity obstacle of the obstacle
        '''

        radius = robot.radius + obstacle.radius
        position = obstacle.position - robot.position
        velocity = obstacle.velocity - robot.velocity
    



def run():

    vo = VelocityObstacle()
    init_positions_obstacle = [[5, 0], [3, 4]]
    # obstacles = []
    destination = np.array([5, 10])
    bot = Robot(init_position = np.array([2, 1]), radius = 2, dt = 0.1, destination=destination)
    
    # obstacle = Robot(init_position = np.array())

if __name__=="__main__":

    run()
    