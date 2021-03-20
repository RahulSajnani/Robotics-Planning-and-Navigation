from sys import platform
import numpy as np
from Agents import *
from Graph import *
from utils import *
from Map import *
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

'''
Authors:

Rahul Sajnani
Amarthya Sasi Kiran
Abhiram Kadiyala

Date: 5th February 2021
'''


class RRT:
    '''
    Class for RRT algorithm
    '''
    def __init__(self, agent, map):
        '''
        Initialization of agent, map, edges, velocities, and nearest neighbor classifier
        '''
        self.agent = agent
        self.map   = map
        self.NNClassifier = NearestNeighbors(n_neighbors = 1) 
        self.graph = Graph()
        self.edge_list = []
        self.wheel_velocity = np.zeros(self.agent.num_wheels)
        self.platform_velocity = np.zeros(2)
        self.wheel_position = []


    def findNearestNode(self, sample_point):
        '''
        Find nearest Neightbor for the sample point
        '''

        self.NNClassifier.fit(self.graph.nodes[:, :2])
        dist, nearest_node_index = self.NNClassifier.kneighbors([sample_point])
        
        return dist[0][0], nearest_node_index[0][0]
    
    def getDirectionVector(self, new_node_direction, nearest_node_index, step_size, round = True):

        direction = np.array(new_node_direction[:2]) - np.array(self.graph.nodes[nearest_node_index][:2])
        direction = direction / np.linalg.norm(direction)
        if round:
            direction = direction.round().astype("int") * step_size
        else:
            direction = direction * step_size

        return direction

    def randomSample(self, step_size, epsilon):
        '''
        Return node after checking for collision and robot constraints
        '''

        new_node_found = False
        
        probability_greedy = np.random.random()

        if probability_greedy > epsilon:
            greedy_strategy = True
        else:
            greedy_strategy = False

        trials = 0
        while not new_node_found:
            
            
            trials += 1
            if trials > 20:
                return None
            
            h = np.random.randint(self.map.map_dimensions[0])
            w = np.random.randint(self.map.map_dimensions[1])
            new_node_direction = [h, w]

            if greedy_strategy:
                new_node_direction = [self.goal_node[0], self.goal_node[1]]

            distance, nearest_node_index = self.findNearestNode(new_node_direction)
            
            # if duplicate node
            if distance == 0:
                continue
            
            # Get direction to the next sampled point
            direction = self.getDirectionVector(new_node_direction, nearest_node_index, step_size)
            new_node, wheel_velocity_vector, platform_velocity_vector, wheel_position = agent.getFuturePositionAfter_dt(self.instantaneous_velocity, self.graph.nodes[nearest_node_index], direction, 1)

            
            # new_node = [direction[0] + self.graph.nodes[nearest_node_index][0], direction[1] + self.graph.nodes[nearest_node_index][1], 0]
            
            # Check for collision
            if not self.collision(new_node, self.graph.nodes[nearest_node_index]):
                new_node_found = True
            else:
                if greedy_strategy:
                    greedy_strategy = False
            
        self.platform_velocity = np.vstack((self.platform_velocity, platform_velocity_vector))
        self.wheel_velocity = np.vstack((self.wheel_velocity, wheel_velocity_vector))
        self.wheel_position.append(wheel_position)

        return new_node, nearest_node_index
    
    def getCollisionRectangle(self, position_1, position_2):

        if position_1 < position_2:
            return position_1.round().astype(int), position_2.round().astype(int)
        else:
            return position_2.round().astype(int), position_1.round().astype(int)

    def collision(self, node, old_node):
        '''
        Function to check collision
        '''

        boundary_check = self.checkBoundary(node)
        if boundary_check:
            return boundary_check

        if old_node is not None:
            vector = np.array(node) - np.array(old_node)
            # x = (vector * np.cos(np.deg2rad(node[2])))[0]
            x = vector[0]
            y = vector[1]
            # y = (vector * np.sin(np.deg2rad(node[2])))[1]

            x_low, x_high = self.getCollisionRectangle(old_node[0], old_node[0] + x)
            y_low, y_high = self.getCollisionRectangle(old_node[1], old_node[1] + y)

            if (self.map.cmap[x_low:x_high + 1, y_low:y_high + 1] == 0).any():
                return True

        if self.map.cmap[node[0], node[1]] != 0:
            return False
        else:
            return True

    def checkBoundary(self, node):
        '''
        Check if bot has exceeded past the map boundary
        '''

        if node[0] < 0 or node[0] > self.map.map_dimensions[0]:
            return True

        if node[1] < 0 or node[1] > self.map.map_dimensions[1]:
            return True

    def plotTrajectory(self, out_dict):

        plt.imshow(map_object.map, cmap="gray")
        plt.plot(init_node[1], init_node[0], 'ro')
        plt.plot(goal_node[1], goal_node[0], 'go')

        for i in range(len(out_dict["nodes"])):
            for j in range(len(out_dict["edges"][i])):
                
                if out_dict["edges"][i][j] < i:
                    continue
                node_1 = out_dict["nodes"][i]
                node_2 = out_dict["nodes"][out_dict["edges"][i][j]]

                plt.plot([node_1[1], node_2[1]], [node_1[0], node_2[0]])

        plt.title("Environment map")
        plt.show()

    def getPathToGoal(self):
        '''
        Get path to goal
        '''
        path, edge_list = rrt_object.graph.findPathToGoal(0, out_dict["goal_index"])
        return path, edge_list
        

    def plotLiveTrajectory(self, edge_list, title, plot_wheels = False):
        '''
        Plot trajectory as they appear
        '''
        
        plt.imshow(self.map.map, cmap="gray")
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        total_edges = len(edge_list)
  
        colors = ["red", "yellow", "magenta"]
        colors_dict = {"red": "r", "yellow": "y","magenta":"m"}
        # Display initial and destination nodes
        plt.plot(self.init_node[1], self.init_node[0], "bo", label="Initial node")
        plt.plot(self.goal_node[1], self.goal_node[0], "go", label="Goal node")
        plt.legend()
        plt.title(title)

        plt.pause(2)

        m = 0

        plot_interval = int(total_edges / 30)
        
        for edge in edge_list:
            node_index_1, node_index_2 = edge
            node_1 = self.graph.nodes[node_index_1]
            node_2 = self.graph.nodes[node_index_2]

            if not plot_wheels:
                # Plot only platform
                
                plt.plot([node_1[1], node_2[1]], [node_1[0], node_2[0]], "r")    
                red_patch = mpatches.Patch(color='red', label="$v_x$ = %.02f" % self.platform_velocity[node_index_1, 1])
                blue_patch = mpatches.Patch(color='blue', label="$v_y$ = %.02f" % self.platform_velocity[node_index_1, 0])
                plt.legend(handles=[red_patch, blue_patch])
            else:
                # Plot wheels only
                wheel_positions_1 = self.wheel_position[node_index_1]
                # print(wheel_positions_1, node_1)
                wheel_positions_2 = self.wheel_position[node_index_2]

                patch_list = []
                i = 0
                for wheel in wheel_positions_1:

                    position_1 = wheel_positions_1[wheel]
                    position_2 = wheel_positions_2[wheel]

                    patch = mpatches.Patch(color=colors[i], label= "$" + wheel + "$ = %.02f" % self.wheel_velocity[node_index_1][i])
                    plt.plot([position_1[1], position_2[1]], [position_1[0], position_2[0]], colors_dict[colors[i]])
                    patch_list.append(patch)
                    i = i + 1
                
                plt.legend(handles = patch_list)
            
            m = m + 1
            if (m % plot_interval == 0):
                plt.pause(0.01)

        if m:
            plt.pause(0.01)
        plt.show()
        

    def visualizeTrajectories(self, opt = 0):
        '''
        Visualize all trajectories

        opt = 0, Path to destination
        opt = 1, RRT tree
        opt = 2, wheel trajectory to destination
        opt = 3, wheel trajectory in RRT tree
        '''
        
        bot = agent.type

        if opt == 0:
            path, edge_list = self.getPathToGoal()
            title = bot + " destination trajectory (platform)"
            self.plotLiveTrajectory(edge_list, title, False)

        elif opt == 1:
            title = bot + " RRT Tree (platform)"
            self.plotLiveTrajectory(self.edge_list, title, False)
        
        elif opt == 2:
            path, edge_list = self.getPathToGoal()
            title = bot + " destination trajectory (wheels) "
            self.plotLiveTrajectory(edge_list, title, True)
        
        elif opt == 3:
            title = bot + " RRT Tree (wheels) "
            self.plotLiveTrajectory(self.edge_list, title, True)

    def run(self, init_node, goal_node, num_iterations = 10, step_size = 1, epsilon = 0.9, instantaneous_velocity = 10):
        '''
        Run RRT algorithm
        '''
        
        # Generate Config map 
        self.map.createConfigMap(self.agent.radius_bot + 1)

        self.instantaneous_velocity = instantaneous_velocity
        
        # Initialize nodes
        self.init_node = init_node
        self.goal_node = goal_node
        self.wheel_position.append(self.agent.getWheelPosition(init_node))

        # Goal node found
        is_goal_found = False

        # Add init note to graph
        self.graph.addNode(init_node)

        for i in range(num_iterations):
            
            # Get new node by randomly sampling
            return_sample = self.randomSample(step_size, epsilon)

            if return_sample is not None:
                new_node, nearest_node_index = return_sample
            else:
                continue
            # Add new node to the graph
            new_node_index = self.graph.addNode(new_node)

            # Add edge to the graph
            self.graph.addEdge(nearest_node_index, new_node_index)

            # Edge preserving edge list to plot later
            self.edge_list.append((nearest_node_index, new_node_index))
            if np.linalg.norm(np.array(self.graph.nodes[new_node_index][:2]) - np.array(goal_node)) < 10:
                print("Goal Reached! ")
                goal_node_index = new_node_index
                is_goal_found = True
                break
        
        if is_goal_found:
            return {"nodes": self.graph.nodes, "edges": self.graph.edges, "is_goal_found": is_goal_found, "goal_index": goal_node_index}
        else:
            return {"nodes": self.graph.nodes, "edges": self.graph.edges, "is_goal_found": is_goal_found}



if __name__ == "__main__":
    '''
    - Record
    - Report writing - Nearly done
    '''

    np.random.seed(4096)
    map_object = Map()
    map_image = plt.imread("./map2.png")[:, :, 0]
    map_object.initializeMap(map_image)

    agent = Agent_holonomic(radius_bot = 3)
    # agent = Agent_non_holonomic(radius_bot = 3)
    rrt_object = RRT(agent, map_object)
    
    init_node = [10, 10, 0]
    goal_node = [420, 420]

    out_dict = rrt_object.run(init_node, goal_node, num_iterations=3000, epsilon=0.4, instantaneous_velocity = 2)
    
    if out_dict["is_goal_found"]:
        rrt_object.visualizeTrajectories(opt = 0)
        rrt_object.visualizeTrajectories(opt = 1)
        rrt_object.visualizeTrajectories(opt = 2)
        rrt_object.visualizeTrajectories(opt = 3)

    
    