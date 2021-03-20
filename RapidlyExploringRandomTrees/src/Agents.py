from sys import platform
import numpy as np
import matplotlib.pyplot as plt
import random

'''
Authors:

Rahul Sajnani
Amarthya Sasi Kiran
Abhiram Kadiyala

Date: 5th February 2021
'''


class Agent_holonomic:
	'''
	Omnidirectional robot class
	'''

	def __init__(self, radius_bot):
		
		self.radius_bot = radius_bot
		self.type = "Omnidirectional robot"
		# Omnidirectional wheels angle.
		self.alpha_2 = np.deg2rad(120)
		self.alpha_3 = np.deg2rad(240)
		self.wheel_radius = 1
		self.num_wheels = 3


	def getFuturePositionAfter_dt(self, velocity, position_old, direction_vector, dt):
		'''
		Get position after applying velocity and time step
		'''
		
		v_x = velocity * direction_vector[0]
		v_y = velocity * direction_vector[1]

		x_new = v_x * dt + position_old[0]
		y_new = v_y * dt + position_old[1]
		theta_new = position_old[2]

		omega = 0

		# Get outputs
		wheel_velocity_vector = self.getWheelVelocity(v_x, v_y, omega, position_old).squeeze()
		position_new = [x_new, y_new, theta_new]
		platform_velocity = np.array([v_x, v_y])
		wheel_position = self.getWheelPosition(position_new)


		return position_new, wheel_velocity_vector, platform_velocity, wheel_position
	
	def getWheelVelocity(self, v_x, v_y, omega, position_old):
		'''
		Function to obtain wheel velocity of omnidirectional drive
		'''

		theta_old = np.deg2rad(position_old[2])
		velocity_vector= np.array([[v_x], [v_y], [omega]])
		
		# Local to global transform
		local_T_global = np.array([[np.cos(theta_old),   			   0,           0],
								   [                0, np.cos(theta_old),    		0],
								   [                0,                 0,    		1]])
		
		# Local to wheel transform
		wheel_T_local = np.array([[ 			   -np.sin(theta_old), 				  np.cos(theta_old), self.radius_bot],
								  [ -np.sin(theta_old + self.alpha_2), np.cos(theta_old + self.alpha_2), self.radius_bot],
								  [ -np.sin(theta_old + self.alpha_3), np.cos(theta_old + self.alpha_3), self.radius_bot]])

		# Apply global to wheel transform to get velocity for each of the wheels in Omnidirectional robot
		wheel_velocity_vector = wheel_T_local @ local_T_global @ velocity_vector
		
		return wheel_velocity_vector
		
	def getWheelPosition(self, position):
		'''
		Get wheel position given position of platform
		'''

		theta = np.deg2rad(position[2])
		x_1 = position[0] + self.radius_bot * np.cos(theta)
		y_1 = position[1] + self.radius_bot * np.sin(theta)
		
		x_2 = position[0] + self.radius_bot * np.cos(theta + self.alpha_2)
		y_2 = position[1] + self.radius_bot * np.sin(theta + self.alpha_2)
		
		x_3 = position[0] + self.radius_bot * np.cos(theta + self.alpha_3)
		y_3 = position[1] + self.radius_bot * np.sin(theta + self.alpha_3)
		
		theta_wheel = np.rad2deg(theta)
		return {"wheel_1": (x_1, y_1, theta_wheel), "wheel_2": (x_2, y_2, theta_wheel + np.rad2deg(self.alpha_2)), "wheel_3": (x_3, y_3, theta_wheel + np.rad2deg(self.alpha_3))}

class Agent_non_holonomic:
	'''
	Differential drive robot class
	'''

	def __init__(self, radius_bot, step_size = 1):
		
		# Bot radius
		self.radius_bot = radius_bot
		self.step_size = step_size
		self.type = "Differential drive"
		# Maximum rotation angle
		self.max_rotation = 15
		self.num_wheels = 2
		
	def getFuturePositionAfter_dt(self, velocity, position_old, direction_vector, dt):
		'''
		Get position after applying velocity and time step
		'''
		
		theta_pts = np.arccos(1 * direction_vector[0] / np.sqrt(direction_vector[0]**2 + direction_vector[1]**2))
		degree_error = (np.rad2deg(theta_pts) - position_old[2])
		
		# Constrain the robot's orientation
		if np.abs(degree_error) < self.max_rotation:
			omega = degree_error
		else:
			omega = np.sign(degree_error) * self.max_rotation


		v_x = velocity * np.cos(np.deg2rad(omega*dt + position_old[2]))
		v_y = velocity * np.sin(np.deg2rad(omega*dt + position_old[2]))
		
		x_new = np.round(v_x * dt + position_old[0]).astype(int)
		y_new = np.round(v_y * dt + position_old[1]).astype(int)
		theta_new = position_old[2] + omega*dt
		
		

		# Outputs
		wheel_velocity_vector = self.getWheelVelocity(velocity, v_x, v_y, omega, position_old).squeeze()
		platform_velocity = np.array([v_x, v_y])
		position_new = [x_new, y_new, theta_new]
		wheel_position = self.getWheelPosition(position_new)

		return position_new, wheel_velocity_vector, platform_velocity, wheel_position
	
	def getWheelVelocity(self, velocity, v_x, v_y, omega, position_old):
		'''
		Get position after applying velocity and time step
		'''
		
		if omega != 0:
			# If angular velocity is not zero, ICC (Instantaneous Center of Curvature) is not infinity 
			R = velocity / omega
			v_l = omega * (R + self.radius_bot)
			v_r = omega * (R - self.radius_bot)
		else:
			v_l = velocity
			v_r = velocity

		wheel_velocity_vector = np.array([[v_l], [v_r]])
		return wheel_velocity_vector

	def getWheelPosition(self, position):
		'''
		Get wheel position given position of platform
		'''

		theta = np.deg2rad(position[2])
		x_1 = position[0] + (self.radius_bot * np.sin(theta))
		y_1 = position[1] + (self.radius_bot * np.cos(theta))
		
		# print(position[0], x_1)
		x_2 = position[0] + (self.radius_bot * np.sin(theta + np.pi))
		y_2 = position[1] + (self.radius_bot * np.cos(theta + np.pi))
		
		theta_wheel = np.rad2deg(theta)
		return {"wheel_1": (x_1, y_1, theta_wheel), "wheel_2": (x_2, y_2, theta_wheel)}


if __name__=="__main__":
    
	agent = Agent_non_holonomic()
	position_new = agent.getFuturePositionAfter_dt(2, (0, 0, -35), (1, 0), 1)
	print(position_new)
