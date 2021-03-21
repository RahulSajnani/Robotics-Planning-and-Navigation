import numpy as np
import cvxpy as cp

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
        print(len(self.obstacles))
        
        
    def getProblemMatrix(self, goal):
        '''
        Get q1 and q2 for the problem
        '''

        A = np.ones((self.steps, self.steps)) * self.delta_t**2
        B = A.copy()

        q1 = 2 * np.ones((self.steps, 1)) * (self.x_start[0] - goal[0]) * self.delta_t
        q2 = 2 * np.ones((self.steps, 1)) * (self.x_start[1] - goal[1]) * self.delta_t

        C1 = (self.x_start[0] - goal[0])**2
        C2 = (self.x_start[1] - goal[1])**2

        return {"A": A, "B": B, "q1": q1, "q2": q2, "C1": C1, "C2": C2}

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

        constraints = self.getVelocityConstraints(X, Y)
        problem = cp.Problem(cp.Minimize( cp.quad_form(X, A) + q1.T @ X + C1 \
                                       +  cp.quad_form(Y, B) + q2.T @ Y + C2),
                                        constraints)
        problem.solve(verbose = verbose)

        # print(X.value, Y.value)

        # Solve using obstacle constraints
        
            
        X_star = np.expand_dims(np.array(X.value), axis = -1)
        Y_star = np.expand_dims(np.array(Y.value), axis = -1)

        X = cp.Variable(self.steps)
        Y = cp.Variable(self.steps)    

        # Obstacle constraints
        constraints = self.getObstacleConstraints(X, Y, X_star, Y_star)
        

        if len(constraints) > 0:
        
            # Velocity constraints
            constraints.extend(self.getVelocityConstraints(X, Y))

            # Optimization problem
            problem = cp.Problem(cp.Minimize( cp.quad_form(X, A) + q1.T @ X + C1 \
                                           +  cp.quad_form(Y, B) + q2.T @ Y + C2),
                                              constraints)

            # Solve the problem
            problem.solve(verbose = verbose)

            print(X.value)
            X_star = np.expand_dims(np.array(X.value), axis = -1)
            Y_star = np.expand_dims(np.array(Y.value), axis = -1)

            # print(X_star, Y_star)

        return X_star, Y_star

    def getVelocityConstraints(self, X, Y):
        '''
        Get velocity constraints
        '''

        h = np.ones(self.steps) * self.v_max
        m = np.zeros(self.steps)

        constraints = [X <= h, 
                       m <= X, 
                       Y <= h,
                       m <= Y]
        return constraints

    def getObstacleConstraints(self, X, Y, X_star, Y_star):
        '''
        Obtain obstacle constraints
        '''

        constraints = []
        
        if len(self.obstacles) > 0:
            
            for obs_i in self.obstacles:
                print(obs_i)
                prob_matrix = self.getProblemMatrix(obs_i)        
                
                A = prob_matrix["A"]
                B = prob_matrix["B"]
                q1 = prob_matrix["q1"] 
                q2 = prob_matrix["q2"] 
                C1 = prob_matrix["C1"]
                C2 = prob_matrix["C2"]

                # print(X - X_star, X_star.shape)
                # print(X_star.shape)
                constraints.append(obs_i[2]**2 <= X_star.T @ A @ X_star + q1.T @ X_star + C1 \
                                    +  Y_star.T @ B @ Y_star + q2.T @ Y_star + C2 \
                                    + (2 * A @ X_star + 2 * q1).T @ (X - X_star.squeeze())   \
                                    + (2 * B @ Y_star + 2 * q2).T @ (Y - Y_star.squeeze())   \
                                    )

        return constraints

if __name__ == "__main__":

    optim = MPCOptimizer(x_start=[0, 0], x_goal=[5, 5], v_max=10, steps=30, obstacles=[[2, 2, 1], [4, 4, 0.5]])
    optim.solve(verbose = False)


        
    
