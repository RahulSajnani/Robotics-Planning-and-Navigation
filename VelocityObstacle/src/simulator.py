import numpy as np
import matplotlib.pyplot as plt
from velocity_obstacle import VelocityObstacle
from agent import Robot
import hydra
from omegaconf import DictConfig, OmegaConf
import matplotlib.patches as mpatches

class Simulator:

    def __init__(self, config):

        self.preprocessConfigs(config)
        self.initializeConfigs()

            
    def preprocessConfigs(self, cfg):
        '''
        Convert configs to numpy arrays
        '''
        self.config = {}
        for key in cfg["env"]:
            self.config[key] = np.array(cfg["env"][key])
        
    def initializeConfigs(self):
        '''
        Initialize bot positions and obstacles
        '''

        # Velocity obstacles class to avoid dyamic obstacles
        self.vo = VelocityObstacle(v_max = self.config["v_max"])
        self.obstacles = []

        # Initialize obstacles
        for i in range(len(self.config["obstacles_position"])):
            obs = Robot(init_position = self.config["obstacles_position"][i], 
                        radius = self.config["obstacles_radius"][i], 
                        dt = self.config["delta_t"], 
                        destination = self.config["obstacles_destination"][i]) 
            self.obstacles.append(obs)
        
        # Initialize robot 
        self.bot = Robot(init_position = self.config["robot_position"], 
                        radius = self.config["robot_radius"], 
                        dt = self.config["delta_t"], 
                        destination=self.config["robot_destination"])
        

    def plotCircle(self, x, y, radius, color = "black"):
        
        circle1 = plt.Circle((x, y), radius, color=color)
        plt.gca().add_patch(circle1)
    
    def run(self):
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        while np.linalg.norm(self.bot.position - self.bot.destination) > self.config["destination_region"]:
            blue_patch = mpatches.Patch(color='blue', label="Robot")
            black_patch = mpatches.Patch(color='black', label="Obstacles")
            plt.legend(handles=[black_patch, blue_patch])
        
            delta_v = self.vo.sampleVelocity(self.bot, self.obstacles)
            
            for obs in self.obstacles:
                obs.step()
                path = obs.getPath()
                plt.plot(path[:, 0], path[:, 1])
                self.plotCircle(obs.position[0], obs.position[1], obs.radius)

            self.bot.step(velocity = delta_v)
            path = self.bot.getPath()
            plt.plot(path[:, 0], path[:, 1])

            self.plotCircle(self.bot.position[0], self.bot.position[1], self.bot.radius, "blue")
            plt.axis("equal")
            plt.title("Velocity obstacle for dynamic obstacle avoidance")
            plt.pause(0.1)
            plt.clf()
        plt.show()
            
        print("Reached goal")



@hydra.main(config_name = "config/config.yml")
def main(cfg):
    sim = Simulator(cfg)
    sim.run()

if __name__=="__main__":
    main()
