import numpy as np
import matplotlib.pyplot as plt
from agent import Robot


class VelocityObstacle:

    def __init__(self, v_min, v_max):
        
        self.velocity_samples = np.linspace(0, v_max, 20)    
        self.angle_samples = np.linspace(0, 2*np.pi, 60)

    def getVelocityObstacleConstraint(self, robot, obstacle, delta_v):
        '''
        Get velocity obstacle of the obstacle
        '''

        radius = robot.radius + obstacle.radius
        r = obstacle.position - robot.position
        v_ro = (robot.velocity - obstacle.velocity)


        v_new = (v_ro + delta_v)
        norm = np.linalg.norm(v_new)
        if norm > 0:
            v_new = v_new / norm

        constraint = (np.dot(r, r) - np.dot(r, v_new)**2 >= radius**2)
        # print(r, v_new)        
        return constraint

    def rotate(self, vector, theta):
        '''
        Rotate vector by theta in 2D
        '''

        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta),  np.cos(theta)]])

        return rotation_matrix @ vector

    def sampleVelocity(self, robot, obstacles):
        '''
        Sample velocity change to move the robot
        '''
        
        delta_init = np.array([1, 0])
        for velocity in self.velocity_samples:
            for angle in self.angle_samples: 
    
                delta_v = self.rotate((delta_init * velocity), angle)
                constraint_satisfied = False
                for obs in obstacles:
                    constraint_satisfied = self.getVelocityObstacleConstraint(robot, obs, delta_v)    
                    if not constraint_satisfied:
                        break
                if not constraint_satisfied:
                    continue

                return delta_v
        print("sad")
        return 0   

if __name__=="__main__":
    pass