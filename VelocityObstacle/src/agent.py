import numpy as np
import matplotlib.pyplot as plt


class Robot:

    def __init__(self, init_position, radius, dt, destination):
        '''
        Initialize the initial position of bot and bot radius
        '''
        self.position = init_position
        self.radius = radius
        self.velocity = 0
        self.dt = dt
        self.destination = destination
        self.setInitialVelocity()

    def setInitialVelocity(self):
        '''
        Set initial velocity towards destination
        '''

        dir_vector = self.destination - self.position

        norm_dir_vector = np.linalg.norm(dir_vector)
        if norm_dir_vector >= 1:
            dir_vector = dir_vector / norm_dir_vector
        self.velocity = dir_vector

    def setVelocity(self, velocity):
        '''
        Set bot velocity
        '''

        self.velocity = velocity

        
    def moveByVelocity_dt(self):
        '''
        Move by velocity and given dt
        
        Param:
            velocity - 2 x 1
            dt       - scalar
        '''   
        
        if np.linalg.norm(self.position - self.destination) < 0.2:
            self.velocity = 0
        self.position = self.position + self.velocity * self.dt

    def getPosition(self):
        '''
        Get bot position 
        '''
        return self.position
    
    def step(self, velocity = None):
        '''
        Take step in the world
        '''

        if velocity is None:
            if np.linalg.norm(self.velocity) != 0:
                self.setInitialVelocity()
        else:
            self.setVelocity(velocity + self.velocity)
        
        self.moveByVelocity_dt()
        self.setInitialVelocity()

if __name__=="__main__":

    destination = np.array([5, 10])
    bot = Robot(init_position = np.array([2, 1]), radius = 2, dt = 0.1, destination=destination)
    