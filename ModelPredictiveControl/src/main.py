import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from plotter import Plotter


'''
Author:
    Rahul Sajnani
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
        self.plotter = Plotter()
        self.trust_region_threshold = 0.1
        self.randomization_threshold = 0.1
        self.max_iter = 10
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

                    constraints.append(obs_i[2]**2 <= self.linearization(X, X_star, A, q1, C1) + self.linearization(Y, Y_star, B, q2, C2))

        return constraints


    def quadraticCost(self, X, A, q1, C1):
        '''
        Compute quadratic cost
        '''

        cost = X.T @ A @ X + q1.T @ X + C1    
        return cost
    
    def linearization(self, X, X_star, A, q1, C1):
        '''
        Linearize around X_star
        '''

        if (X.shape == X_star.shape):
            linearized_cost = self.quadraticCost(X_star, A, q1, C1) + (2 * A @ X_star + q1).T @ (X - X_star) 
        else:
            linearized_cost = self.quadraticCost(X_star, A, q1, C1) + (2 * A @ X_star + q1).T @ (X - X_star.reshape(X.shape)) 

        return linearized_cost


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

        X_star = np.expand_dims(np.array(X.value), axis = -1) 
        Y_star = np.expand_dims(np.array(Y.value), axis = -1)
       
        # Solve using obstacle constraints if obstacles are present
        if len(self.obstacles) > 0:
            

            trust_region = False
            while(not trust_region):

                X_star = X_star + np.random.random(X_star.shape) * self.randomization_threshold
                Y_star = Y_star + np.random.random(Y_star.shape) * self.randomization_threshold

            
                X = cp.Variable(self.steps)
                Y = cp.Variable(self.steps)    
                print(X.shape)
                # Obstacle constraints
                constraints = self.getObstacleConstraints(X, Y, X_star, Y_star)
                
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
                problem = cp.Problem(cp.Minimize(20 * (cp.quad_form(X, A) + q1.T @ X + C1 \
                                            +  cp.quad_form(Y, B) + q2.T @ Y + C2)),
                                                constraints)

                # Solve the problem
                problem.solve(verbose = verbose, warm_start = True, solver = cp.SCS)

                if X.value is None:
                    print("No solution")
                    self.randomization_threshold += 0.1

                    if self.randomization_threshold > 0.4:
                        print("Please increase number of steps")
                        exit()
                    continue
                    # self.steps += 5
                    # self.v_max += 0.05
                
                X_star_new = np.expand_dims(np.array(X.value), axis = -1) 
                Y_star_new = np.expand_dims(np.array(Y.value), axis = -1)
                
                # Trust region check
                error = (np.mean(np.abs(X_star - X_star_new)) + np.mean(np.abs(Y_star - Y_star_new))) 
                if  error <= 2 * self.trust_region_threshold:
                    trust_region = True
                
                print(error, "#"*100)
                X_star = X_star_new
                Y_star = Y_star_new
                
                # print(X_star, Y_star)
        self.plot(X_star, Y_star)

        return X_star, Y_star

    def getPath(self, X, Y):
        '''
        Plot the trajectory
        '''

        # print(X)
        position_X = self.x_start[0] + np.cumsum(X) * self.delta_t
        position_Y = self.x_start[1] + np.cumsum(Y) * self.delta_t

        position_X = np.hstack((self.x_start[0], position_X))
        position_Y = np.hstack((self.x_start[1], position_Y))

        return position_X, position_Y

    def plot(self, X_star, Y_star):
        
        path_x, path_y = self.getPath(X_star, Y_star)
        self.plotter.plot(self.x_start, self.x_goal, self.obstacles, path_x, path_y)

        
        

if __name__ == "__main__":

    ############### Settings ##############################
    x_start = [0, 0] 
    x_goal = [8, 13] 
    v_max = 0.5
    steps = 40
    delta_t = 1
    obstacles = [[2, 8, 1],
                 [3, 4, 1], 
                 [4, 2, 0.5], 
                 [2, 2, 0.5], 
                 [8, 6, 1], 
                 [8, 2, 1],
                 ]

    # obstacles = []
    #######################################################


    optim = MPCOptimizer(x_start=x_start, x_goal=x_goal, v_max=v_max, steps=steps, delta_t = delta_t, obstacles=obstacles)
    X_star, Y_star = optim.solve(verbose = False)
    print(X_star, Y_star)
    
