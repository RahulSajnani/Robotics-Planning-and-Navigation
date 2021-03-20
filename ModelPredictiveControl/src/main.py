import numpy as np
import cvxpy as cp

'''
Authors:
    Rahul Sajnani
    Shirish Chandra
Date:
    21st March 2021
'''


class MPCOptimization:
    def __init__(self, x_start, x_goal, v_max, steps = 3, delta_t = 0.1):
        '''
        Setting start, goal, number of steps to take and minimum time step duration
        '''

        self.x_start = x_start
        self.x_goal = x_goal
        self.steps = steps
        self.delta_t = delta_t
        self.v_max = v_max
        
        
    def getProblemMatrix(self):
        '''
        Get q1 and q2 for the problem
        '''

        A = np.ones((self.steps, self.steps)) * self.delta_t**2
        B = A.copy()

        q1 = 2 * np.ones((self.steps, 1)) * (self.x_start[0] - self.x_goal[0]) * self.delta_t
        q2 = 2 * np.ones((self.steps, 1)) * (self.x_start[1] - self.x_goal[1]) * self.delta_t

        C1 = (self.x_start[0] - self.x_goal[0])**2
        C2 = (self.x_start[1] - self.x_goal[1])**2

        return {"A": A, "B": B, "q1": q1, "q2": q2, "C1": C1, "C2": C2}

    def solve(self):
        '''
        Solve the Model Predictive control optimization problem
        ''' 

        X = cp.Variable(self.steps)
        Y = cp.Variable(self.steps)

        prob_martix = self.getProblemMatrix()
        
        A = prob_martix["A"]
        B = prob_martix["B"]
        q1 = prob_martix["q1"]
        q2 = prob_martix["q2"]
        C1 = prob_martix["C1"]
        C2 = prob_martix["C2"]

        h = np.ones(self.steps) * self.v_max
        m = np.zeros(self.steps)

        problem = cp.Problem(cp.Minimize( cp.quad_form(X, A) + q1.T @ X + C1 \
                                       +  cp.quad_form(Y, B) + q2.T @ Y + C2),
                                        [X <= h,
                                         m <= X,
                                         Y <= h,
                                         m <= Y])
        problem.solve()

        print(X.value, Y.value)

if __name__ == "__main__":

    optim = MPCOptimization(x_start=[0, 0], x_goal=[2, 2], v_max=10, steps=4)
    optim.solve()
        
    
