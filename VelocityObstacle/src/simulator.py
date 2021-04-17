import numpy as np
import matplotlib.pyplot as plt
from velocity_obstacle import VelocityObstacle
from agent import Robot
import hydra
from omegaconf import DictConfig, OmegaConf

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
        self.vo = VelocityObstacle(v_min = self.config["v_min"], v_max = self.config["v_max"])
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
        
    def run(self):

        while np.linalg.norm(self.bot.position - self.bot.destination) > self.config["destination_region"]:
            

            
            delta_v = self.vo.sampleVelocity(self.bot, self.obstacles)
            if np.linalg.norm(delta_v) > 0:
                print("change of path", delta_v)
                print("#"*100)
                exit()
            for obs in self.obstacles:
                obs.step()
                print("Obstacle dest: ", obs.destination, " position ", obs.position)
            
            self.bot.step(velocity = delta_v)
            print("Bot dest: ", self.bot.destination, " position ", self.bot.position)
            



@hydra.main(config_name = "config/config.yml")
def main(cfg):
    sim = Simulator(cfg)
    sim.run()

if __name__=="__main__":
    main()
