import numpy as np
import matplotlib.pyplot as plt


class Robot:

    def __init__(self, init_position, radius):
        '''
        Initialize the initial position of bot and bot radius
        '''
       
        self.position = init_position
        self.radius = radius
    
    def moveByVelocity_dt(self, velocity, dt):
        '''
        Move by velocity and given dt
        
        Param:
            velocity - 2 x 1
            dt       - scalar
        '''
       
        self.position = self.position + velocity * dt

    def getPosition(self):
        '''
        Get bot position 
        '''
        
        return self.position
    
    


if __name__=="__main__":

    bot = Robot(init_position = np.array([2, 1]), radius = 2)
    