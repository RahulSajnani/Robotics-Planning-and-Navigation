import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
# from plotter import Plotter


'''
Authors:
    Rahul Sajnani
    Shirish Chandra
Date:
    21st March 2021
'''


class MPCOptimizer:

    def __init__(self, x_start, x_goal, v_max, steps = 3, delta_t = 0.1, obstacles = []):
        '''
        Setting start, goal, number of steps to take and minimum time step duration
        '''

        self.x_start = x_start
        self.x_goal = x_goal
        self.steps = steps
        self.delta_t = delta_t
        self.v_max = v_max
        self.obstacles = obstacles
        # print(len(self.obstacles))
        
        
    def getProblemMatrix(self, goal, time_step = -1):
        '''
        Get q1 and q2 for the problem
        '''

        if time_step == -1:
            time_step = self.steps

        ones_vector = np.zeros((self.steps, 1))
        ones_vector[:time_step + 1, :] = 1

        A = (ones_vector @ ones_vector.T) * self.delta_t**2
        # print(A.shape)
        B = A.copy()

        
        q1 = 2 * ones_vector * (self.x_start[0] - goal[0]) * self.delta_t
        q2 = 2 * ones_vector * (self.x_start[1] - goal[1]) * self.delta_t

        C1 = (self.x_start[0] - goal[0])**2
        C2 = (self.x_start[1] - goal[1])**2

        return {"A": A, "B": B, "q1": q1, "q2": q2, "C1": C1, "C2": C2}

    def getVelocityConstraints(self, X, Y):
        '''
        Get velocity constraints
        '''

        h = np.ones(self.steps) * self.v_max
        m = np.zeros(self.steps)

        constraints = [X <= h, 
                      -h <= X, 
                       Y <= h,
                      -h <= Y]
        return constraints

    def getObstacleConstraints(self, X, Y, X_star, Y_star):
        '''
        Obtain obstacle constraints
        '''

        constraints = []
        
        if len(self.obstacles) > 0:    
            for obs_i in self.obstacles:
                for t in range(self.steps):
                    prob_matrix = self.getProblemMatrix(obs_i, t)        
                    
                    A = prob_matrix["A"]
                    B = prob_matrix["B"]
                    q1 = prob_matrix["q1"] 
                    q2 = prob_matrix["q2"] 
                    C1 = prob_matrix["C1"]
                    C2 = prob_matrix["C2"]

                    q1_new = q1.copy() #+ 2 * self.delta_t**2 * np.sum(X_star)
                    q2_new = q2.copy() #+ 2 * self.delta_t**2 * np.sum(Y_star)
                
                    constraints.append(obs_i[2]**2 <= X_star.T @ A @ X_star + q1.T @ X_star + C1 \
                                                   +  Y_star.T @ B @ Y_star + q2.T @ Y_star + C2 \
                                                   + (2 * A @ X_star + q1_new).T @ (X - X_star.squeeze())   \
                                                   + (2 * B @ Y_star + q2_new).T @ (Y - Y_star.squeeze())   \
                                                    )

        return constraints

    def solve(self, verbose = False):
        '''
        Solve the Model Predictive control optimization problem
        ''' 

        X = cp.Variable(self.steps)
        Y = cp.Variable(self.steps)

        prob_matrix = self.getProblemMatrix(self.x_goal)
        
        A = prob_matrix["A"]
        B = prob_matrix["B"]
        q1 = prob_matrix["q1"]
        q2 = prob_matrix["q2"]
        C1 = prob_matrix["C1"]
        C2 = prob_matrix["C2"]

        # print(A.shape, q1.shape)
        constraints = self.getVelocityConstraints(X, Y)
        problem = cp.Problem(cp.Minimize( cp.quad_form(X, A) + q1.T @ X + C1 \
                                       +  cp.quad_form(Y, B) + q2.T @ Y + C2),
                                        constraints)
        problem.solve(verbose = verbose)

        # print(X.value, Y.value)

        # Solve using obstacle constraints
        
            
        X_star = np.expand_dims(np.array(X.value), axis = -1) 
        Y_star = np.expand_dims(np.array(Y.value), axis = -1)
        # print(X_star.shape)
        X_star = X_star + np.random.random(X_star.shape) * 0.1
        Y_star = Y_star + np.random.random(Y_star.shape) * 0.1

        # print(X_star)
        X = cp.Variable(self.steps)
        Y = cp.Variable(self.steps)    

        # Obstacle constraints
        constraints = self.getObstacleConstraints(X, Y, X_star, Y_star)
        

        if len(constraints) > 0:
        
            prob_matrix = self.getProblemMatrix(self.x_goal)
        
            A = prob_matrix["A"]
            B = prob_matrix["B"]
            q1 = prob_matrix["q1"]
            q2 = prob_matrix["q2"]
            C1 = prob_matrix["C1"]
            C2 = prob_matrix["C2"]
            # Velocity constraints
            constraints.extend(self.getVelocityConstraints(X, Y))

            # Optimization problem
            problem = cp.Problem(cp.Minimize( cp.quad_form(X, A) + q1.T @ X + C1 \
                                           +  cp.quad_form(Y, B) + q2.T @ Y + C2),
                                              constraints)

            # Solve the problem
            problem.solve(verbose = verbose)

            # print(X.value)
            X_star = np.expand_dims(np.array(X.value), axis = -1) 
            Y_star = np.expand_dims(np.array(Y.value), axis = -1)

            print(X_star, Y_star)
            self.plot(X_star, Y_star)

        return X_star, Y_star

    
    
    def plot(self, X, Y):
        '''
        Plot the trajectory
        '''

        # print(X)
        position_X = self.x_start[0] + np.cumsum(X) * self.delta_t
        position_Y = self.x_start[1] + np.cumsum(Y) * self.delta_t

        position_X = np.hstack((self.x_start[0], position_X))
        position_Y = np.hstack((self.x_start[1], position_Y))

    
        plt.plot(position_X[:], position_Y[:])
        plt.plot(self.x_start[0], self.x_start[1], "ro")
        plt.plot(self.x_goal[0], self.x_goal[1], "go")

        for obs in self.obstacles:
            plt.plot(obs[0], obs[1], 'ro', markersize = obs[2])
        
        plt.show()

        

if __name__ == "__main__":

    optim = MPCOptimizer(x_start=[0, 0], x_goal=[6, 12], v_max=3, steps=10, delta_t = 1, obstacles=[[2, 5, 1],[3, 4, 1], [4, 2, 0.5], [2, 2, 0.5]])
    optim.solve(verbose = True)


        
    
