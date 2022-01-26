from argparse import ArgumentParser
import numpy as np
import csv
import time
from matplotlib import pyplot as plt
import random
import math
from gym.spaces import Box
from gym.utils import seeding
from .tools import *

class EnvAirsim():
	def __init__(self, Quadrotor_client, num_states, num_actions):
		self.client = Quadrotor_client
		self.client.confirmConnection()
		self.client.enableApiControl(True)
		self.is_controller = True

		self.num_states = num_states
		self.num_actions = num_actions
		self.observation_space = Box(low=-1.0,high=1.0,shape=(num_states,))
		self.action_space = Box(low=-1.0,high=1.0,shape=(num_actions,))
		self.num_envs = 1
		self.reward_range = (-2.0, 2.0) 
		self.metadata = {}
		self.count = 0
		self.yaw_history = [0.]
		self.throttle_buffer = [0. for i in range(30)]
		self.angular_orientation = []
		self.rotation = 0
		self.initial_time = 0.
		self.rotation_direction = 0.
		self.inclination = 0.
		self.upward_angle = 0.
		self.t_idx = 0
		self.idx = 0
		self.dist = 1.
		self.prev_dist = 1.
		self.action = [0., 0., 0.]
		self.prev_action = [0., 0., 0.]
		self.prev_phi_s = 0.
		self.threshold1 = 2.0
		self.threshold2 = 10.0
		self.observation = []

		state = self.get_state()
		self.initial_position = [state[0], state[1], state[2]]
		self.initial_orientation = [state[6], state[7], state[8], state[9]]
		self.x = 0.
		self.y = 0.
		self.z = 0.
		self.vx = 0.
		self.vy = 0.
		self.vz = 0.
		self.phi = 0.
		self.theta = 0.
		self.psi = 0.
		self.vphi = 0.
		self.vtheta = 0.
		self.vpsi = 0.

		self.vx_norm = 0.
		self.vy_norm = 0.
		self.vz_norm = 0.
		self.ph_normi = 0.
		self.theta_norm = 0.
		self.psi_norm = 0.
		self.vphi_norm = 0.
		self.vtheta_norm = 0.
		self.vpsi_norm = 0.

		self.err_x = 0.
		self.err_y = 0.
		self.err_z = 0.
		self.err_psi = 0.
		self.psi_r = 0.

		self.r_t = []
		self.r_x = []
		self.r_y = []
		self.r_z = []
		self.r_vx = []
		self.r_vy = []
		self.r_vz = []
		self.r_yaw = []
		
		self.r_input = [0.,0.,0.]

		self.T = 0.003
		self.PInit = 0.0000001


	def seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return seed

	def find_orth(self):
		pnt = np.array([self.x,self.y,self.z])
		dist_arr = np.array([])
		dist_min = 10000000.
		# for i in range(len(self.r_t)):
		if self.idx < 100:
			l = 0
		else:
			l = self.idx - 98
		
		if self.idx > len(self.r_x)-100:
			u = len(self.r_x)
		else:
			u = self.idx + 98


		for i in np.linspace(l,u,u-l+1):
			i = int(i)
			dist = np.sqrt(np.sum(np.square(pnt-np.array([self.r_x[i],self.r_y[i],self.r_z[i]]))))
			if dist < dist_min:
				dist_min = dist
				idx_min = i
		return idx_min, dist_min

	def set_initial_time(self):
		self.initial_time = self.get_state()[-1]

	def get_initial_state(self):
		state_current = self.get_state()
		Initial_state = [state_current[0], state_current[1], state_current[2], \
			state_current[3], state_current[4], state_current[5], \
			state_current[6], state_current[7], state_current[8], \
			state_current[10], state_current[11], state_current[12], \
			
			0., 0.,	0., 0., \
			0., 0., 0., 0., \

			0., 0., 0., \
			0., 0., 0., \
			0., 0., 0., \
			0., 0., 0., \

			0., 0., 0., \
			0., 0., 0., \

			0., 0., 0., \
			0., 0., 0., \
			0., 0., 0., \

			self.PInit, 0., 0., \
			0., self.PInit, 0., \
			0., 0., self.PInit, \

			0., 0., 0.	]
		# self.r_input = [state_current[6],state_current[7],state_current[8]]

		return Initial_state

	def get_state(self):
		state = self.client.getMultirotorState()
		linear_position = state.kinematics_estimated.position
		linear_velocity = state.kinematics_estimated.linear_velocity
		angular_angle = state.kinematics_estimated.orientation
		phi = angular_angle.x_val
		theta = angular_angle.y_val
		psi = angular_angle.z_val
		w = angular_angle.w_val
		# angle = orientation_conversion(phi, theta, psi, w, self.yaw_history, self.rotation)
		angle = quaternion2euler(phi, theta, psi, w)
		angular_velocity = state.kinematics_estimated.angular_velocity

		self.x = linear_position.x_val 
		self.y = linear_position.y_val
		self.z = linear_position.z_val
		self.vx = linear_velocity.x_val
		self.vy = linear_velocity.y_val
		self.vz = linear_velocity.z_val
		self.phi = angle[0]
		self.theta = angle[1]
		self.psi = angle[2]
		self.vphi = angular_velocity.x_val
		self.vtheta = angular_velocity.y_val
		self.vpsi = angular_velocity.z_val
		self.t = state.timestamp
		state_current = [ self.x, self.y, self.z, self.vx, self.vy, self.vz, \
						 self.phi, self.theta, self.psi, self.vphi, self.vtheta, self.vpsi, \
						 self.t ]
		return state_current

	def get_state_norm(self):
		self.vx_norm = self.vx/20.
		self.vy_norm = self.vy/20.
		self.vz_norm = self.vz/20.
		self.phi_norm = self.phi/np.pi
		self.theta_norm = self.theta/np.pi
		self.psi_norm = self.psi/np.pi
		self.vphi_norm = self.vphi/(np.pi*10)
		self.vtheta_norm = self.vtheta/(np.pi*10)
		self.vpsi_norm = self.vpsi/(np.pi*10)

		state_current_norm = [ self.x, self.y, self.z, self.vx_norm, self.vy_norm, self.vz_norm, \
						 self.phi_norm, self.theta_norm, self.psi_norm, self.vphi_norm, self.vtheta_norm, self.vpsi_norm, \
						 self.t ]
		return state_current_norm

	def calculate_error(self):
		self.err_psi = abs(self.psi_norm-self.psi_r)
		self.err_x = abs(self.r_x[self.idx] - self.x)
		self.err_y = abs(self.r_y[self.idx] - self.y)
		self.err_z = abs(self.r_z[self.idx] - self.z)
		# self.err_vx = abs(self.r_vx[self.idx+5] - self.vx)
		# self.err_vy = abs(self.r_vy[self.idx+5] - self.vy)
		# self.err_vz = abs(self.r_vz[self.idx+5] - self.vz)
		
	def get_reference(self):
		self.r_t = []
		self.r_x = []
		self.r_y = []
		self.r_z = []
		self.r_vx = []
		self.r_vy = []
		self.r_vz = []
		self.r_yaw = []
		scenario = [0]
		scenario = scenario[random.randint(0,len(scenario)-1)]
		# scenario = 0
		if scenario == 0:
			with open('C:/Users/lee/Desktop/Projects/202008/DDPG_quadrotor_airsim/airsim_env/circle_4m.csv', 'r') as csvFile:
				reader = csv.reader(csvFile)
				for row in reader:
					self.r_t += [float(row[0])]
					self.r_x += [float(row[1])]
					self.r_y += [float(row[2])]
					self.r_z += [float(row[3])]
					self.r_vx += [float(row[4])]
					self.r_vy += [float(row[5])]
					self.r_vz += [float(row[6])]
					self.r_yaw += [float(row[7])]
			csvFile.close()

		# Circular
		if scenario == 1:
			sign = [-1.,1.]
			sign = sign[random.randint(0,1)]
			a = sign * random.uniform(10.,20.)
			rotation = random.uniform(-180.,180.)
			for i in range(1024*4+100):
				self.r_t += [float(0.003*(i+1))]
				self.r_x += [float(a*np.sin(0.1*0.003*i))]
				self.r_y += [float(a*np.cos(0.1*0.003*i)-a)]
				self.r_z += [-1.0]
				self.r_vx += [float(0.1*0.003*a*np.cos(0.1*0.003*i))]
				self.r_vy += [float(-0.1*0.003*a*np.sin(0.1*0.003*i))]
				self.r_vz += [float(0.)]
				self.r_yaw += [float(0.)]

			# for i in range(len(self.r_t)):
			# 	self.r_x[i] = np.cos(rotation)*self.r_x[i] - np.sin(rotation)*self.r_y[i]
			# 	self.r_y[i] = np.sin(rotation)*self.r_x[i] + np.cos(rotation)*self.r_y[i]
			# 	self.r_vx[i] = np.cos(rotation)*self.r_vx[i] - np.sin(rotation)*self.r_vy[i]
			# 	self.r_vx[i] = np.sin(rotation)*self.r_vx[i] + np.cos(rotation)*self.r_vy[i]

		# Rotated Straight Line

		if scenario == 2:
			sign = [-1.,1.]
			sign = sign[random.randint(0,1)]
			speed = sign * random.uniform(1.0,1.5)
			self.rotation_direction = random.uniform(0.,180.)/180.*np.pi
			for i in range(1024*4+100):
				t = i * 0.003
				self.r_t += [float(t)]
				self.r_x += [float(speed*t)]
				self.r_y += [float(0.)]
				self.r_z += [-1.0]
				self.r_vx += [float(speed)]
				self.r_vy += [float(0.)]
				self.r_vz += [float(0.)]
				self.r_yaw += [float(0.)]

			for i in range(len(self.r_t)):
				self.r_x[i] = np.cos(self.rotation_direction)*self.r_x[i] - np.sin(self.rotation_direction)*self.r_y[i]
				self.r_y[i] = np.sin(self.rotation_direction)*self.r_x[i] + np.cos(self.rotation_direction)*self.r_y[i]
				self.r_vx[i] = np.cos(self.rotation_direction)*self.r_vx[i] - np.sin(self.rotation_direction)*self.r_vy[i]
				self.r_vx[i] = np.sin(self.rotation_direction)*self.r_vx[i] + np.cos(self.rotation_direction)*self.r_vy[i]

		if scenario == 3:
			sign = [-1.,1.]
			sign = sign[random.randint(0,1)]
			speed = 1.0
			self.inclination = sign*random.uniform(-0.0001, 0.0001)
			self.inclination = 0.
			self.upward_angle = np.tan(self.inclination/speed)
			self.rotation_direction = 0.
			t = np.linspace(-0.003*1024,-0.003*1024+0.003*1024*6,1024*5)
			self.r_t = t
			self.r_x = speed*np.array(self.r_t)
			self.r_y = [float(0.) for i in range(1024*5)]
			self.r_z = -1.5+self.inclination*np.array(self.r_t)
			self.r_vx = [float(speed) for i in range(1024*5)]
			self.r_vy = [float(0.) for i in range(1024*5)]
			self.r_vz = [float(self.inclination) for i in range(1024*5)]
			self.r_yaw = [float(0.) for i in range(1024*5)]
			for i in range(len(self.r_t)):
				self.r_x[i] = np.cos(self.rotation_direction)*self.r_x[i] - np.sin(self.rotation_direction)*self.r_y[i]
				self.r_y[i] = np.sin(self.rotation_direction)*self.r_x[i] + np.cos(self.rotation_direction)*self.r_y[i]
				self.r_vx[i] = np.cos(self.rotation_direction)*self.r_vx[i] - np.sin(self.rotation_direction)*self.r_vy[i]
				self.r_vx[i] = np.sin(self.rotation_direction)*self.r_vx[i] + np.cos(self.rotation_direction)*self.r_vy[i]

		return self.r_t, self.r_x, self.r_y, self.r_z, self.r_vx, self.r_vy, self.r_vz, self.r_yaw

	
	def get_obs(self):
		err_x_a = (self.r_x[self.idx] - self.x)/self.threshold1
		err_y_a = (self.r_y[self.idx] - self.y)/self.threshold1
		err_z_a = (self.r_z[self.idx] - self.z)/self.threshold1
		# err_x_b = (self.r_x[self.idx+15] - self.x)/self.threshold1
		# err_y_b = (self.r_y[self.idx+15] - self.y)/self.threshold1
		# err_z_b = (self.r_z[self.idx+15] - self.z)/self.threshold1
		# err_x_c = (self.r_x[self.idx+30] - self.x)/self.threshold1
		# err_y_c = (self.r_y[self.idx+30] - self.y)/self.threshold1
		# err_z_c = (self.r_z[self.idx+30] - self.z)/self.threshold1

		# err_psi = self.psi_norm-self.psi_r

		## Second Trial
		# observation = [err_x_a, err_y_a, err_z_a, err_x_b, err_y_b, err_z_b, err_psi, \
		# 			self.vx, self.vy, self.vz, \
		# 			self.phi_norm, self.theta_norm, self.psi_norm, self.vphi_norm, self.vtheta_norm, self.vpsi_norm]

		## Third Trial
		# observation = [err_x_a, err_y_a, err_z_a, err_x_b, err_y_b, err_z_b, err_x_c, err_y_c, err_z_c, err_psi, \
		# 			self.vx, self.vy, self.vz, \
		# 			self.phi_norm, self.theta_norm, self.psi_norm, self.vphi_norm, self.vtheta_norm, self.vpsi_norm]
		observation = [err_x_a, err_y_a, err_z_a, self.action[0], self.action[1], self.action[2]]
		# observation = np.append(observation, np.array(self.prev_action))
		return observation

	def get_reward(self, done_type):
		alp_x = 1./(self.threshold1**2+0.001) * 10.
		alp_y = 1./(self.threshold1**2+0.001) * 10.
		alp_z = 1./(self.threshold1**2+0.001) * 10.
		alp_vx = 1./10.
		alp_vy = 1./10.
		alp_vz = 1./10.
		# alp_phi = 1./np.pi
		# alp_theta = 1./np.pi
		alp_psi = 2.

		alp_a1 = 0.01
		alp_a2 = 0.01
		alp_a3 = 0.01
		alp_a4 = 0.01

		# r_state = - sum([alp_x*(self.err_x**2), alp_y*(self.err_y**2), alp_z*(self.err_z**2), alp_vx*(self.vx_norm**2), alp_vy*(self.vy_norm**2), alp_vz*(self.vz_norm**2), alp_psi*(self.err_psi**2)])
		## First Success
		# alp_psi = 0.1 
		# r_position = sum([0.1-0.1*0.25*(self.err_x**2), 0.1*-0.1*0.25*(self.err_y**2), 0.1-0.1*0.25*(self.err_z**2)])
		# r_state = sum([-0.1*32400./78178.*(self.phi**2), -0.1*32400./78178.*(self.theta**2), -alp_vx*(self.vx_norm**2), -alp_vy*(self.vy_norm**2), -alp_vz*(self.vz_norm**2), -alp_psi*(self.err_psi**2)])
		
		## Second Trial
		# r_position = sum([norm_dist(self.err_x,mu=0,sig=0.5), norm_dist(self.err_y,mu=0,sig=0.5), norm_dist(self.err_z,mu=0,sig=0.5)])/5.
		# r_state = sum([norm_dist(self.phi_norm,mu=0,sig=0.5)-norm_dist(0,mu=0,sig=0.5), norm_dist(self.theta_norm,mu=0,sig=0.5)-norm_dist(0,mu=0,sig=0.5), -alp_vx*(self.vx_norm**2), -alp_vy*(self.vy_norm**2), -alp_vz*(self.vz_norm**2), -alp_psi*(self.err_psi**2)])
		# r_action = sum([-alp_a1*(self.action[0]**2), -alp_a2*(self.action[1]**2), -alp_a3*(self.action[2]**2), -alp_a4*(self.action[3]**2)])
		# r_done = 0.

		# ## Tird Trial
		# r_position = sum([norm_dist(self.err_y,mu=0,sig=0.5), norm_dist(self.err_z,mu=0,sig=0.5)])/3.
		# r_state = sum([norm_dist(self.phi_norm,mu=0,sig=0.5)-norm_dist(0,mu=0,sig=0.5), norm_dist(self.theta_norm,mu=0,sig=0.5)-norm_dist(0,mu=0,sig=0.5), -alp_psi*abs(self.err_psi)])
		# r_action = sum([-alp_a1*(self.action[0]**2), -alp_a2*(self.action[1]**2), -alp_a3*(self.action[2]**2), -alp_a4*(self.action[3]**2)])
		# r_done = 0.

		# gamma = 0.9991
		if self.idx == self.prev_phi_s:
			bonus = 0.2
		elif self.idx > self.prev_phi_s:
			bonus = 0.7
		else:
			bonus = 0.
		
		phi_s = self.idx+bonus
		# F = (phi_s/self.dist - self.prev_phi_s/self.prev_dist)
		F = (phi_s - self.prev_phi_s)*self.prev_dist/self.dist

		# gamma = 0.99
		# phi_s = np.clip(1./self.dist/50.,-1,1)*(self.idx+1)/len(self.r_x)
		# F = phi_s*gamma - self.prev_phi_s

		self.prev_phi_s = self.idx

		
		# if F > 0.:
		# 	F = 0.5
		# if F < 0:
		# 	F = -0.5
		# else:
		# 	F = 0.

		reward_w = - (pow(self.phi_norm,2) + pow(self.theta_norm,2) + pow(self.psi_norm,2))*10.
		reward_vw = - (pow(self.vphi_norm,2) + pow(self.vtheta_norm,2) + pow(self.vpsi_norm,2))*7.
		reward_err = - (pow(self.err_x,2) + pow(self.err_y,2) + pow(self.err_z,2))*0.01
		reward = F + reward_err + reward_w + reward_vw

		# reward = np.clip(sum([r_position, r_state, r_action, F, r_done]),self.reward_range[0],self.reward_range[1])
		# print('U_a: %i, R_p: %.5f, R_s: %.5f, R_a: %.5f F: %.2f, TOTAL: %.5f | action : [%.3f, %.3f, %.3f, %.3f]'%(self.upward_angle,r_position,r_state,r_action,F,reward,self.action[0],self.action[1],self.action[2],self.action[3]))
		# print('R_s: %.3f, x: %.5f, y: %.5f, z: %.5f, x2: %.5f, y2: %.5f, z2: %.5f, psi: %.5f'%(r_state,alp_x*self.err_x**2, alp_y*self.err_y**2, alp_z*self.err_z**2, alp_vx*self.vx_norm**2, alp_vy*self.vy_norm**2, alp_vz*self.vz_norm**2, alp_psi*self.err_psi**2))
		# print('x: %.3f, y: %.3f, z: %.3f, x2: %.3f, y2: %.3f, z2: %.3f, psi: %.3f'%(self.err_x, self.err_y, self.err_z, self.vx_norm, self.vy_norm, self.vz_norm, self.err_psi))
		print("%i | %.4f | %.4f | %.4f | %.4f" %(self.idx, F, reward_err, reward_w, reward_vw))
		# print('TOTAL: %.5f | action : [%.3f, %.3f, %.3f]'%(reward,self.action[0],self.action[1],self.action[2]))
		return reward*0.05

	def reset(self):
		self.client.armDisarm(True)

		# Reset Some Parameters
		self.yaw_history = [0.]
		self.angular_orientation = [] 
		self.rotation = 0
		self.initial_time = 0.
		self.dist = 1.
		self.prev_dist = 1.
		
		self.rotation_direction = 0.
		self.inclination = 0.
		self.upward_angle = 0.
		self.prev_phi_s = 0.

		self.t_idx = 0
		self.idx = 0
		self.action = [0., 0., 0.]
		self.prev_action = [0.,0.,0.]
		self.count = 0

		self.err_psi = 0.
		self.err_x = 0.
		self.err_y = 0.
		self.err_z = 0.
		self.err_vx = 0.
		self.err_vy = 0.
		self.err_vz = 0.

		self.client.reset()
		self.client.confirmConnection()
		self.client.enableApiControl(True)
		
		# Set to initial Position
		pose = self.client.simGetVehiclePose()
		pose.orientation.x_val = 0.
		pose.orientation.y_val = 0.
		pose.orientation.z_val = 0.
		pose.orientation.w_val = 1.
		pose.position.x_val = 0.
		pose.position.y_val = 0.
		pose.position.z_val = -1.
		self.client.simSetVehiclePose(pose, True)
		
		# Take Off
		self.client.simPause(True)
		initX = 0.
		initY = 0.
		initZ = -5.
		
		# Change Initial Position
		self.client.simPause(False)
		print('... Take Off ...')
		self.client.moveToPositionAsync(initX, initY, initZ, 1.5).join()
		# self.client.moveByAngleZAsync(random.uniform(-np.pi*0.1,np.pi*0.1), random.uniform(-np.pi*0.1,np.pi*0.1), initZ, random.uniform(-np.pi*0.25,np.pi*0.25), 0.5).join()

		# self.client.moveByAngleZAsync(0., 0., initZ + random.uniform(-0.1,0.1), 0., 1.1).join()
		# self.client.moveByVelocityAsync(0.5, 0, 0, 1.0).join()
		self.client.simPause(True)
		time.sleep(0.05)

		# Get Reference
		# _,_,_,_,_,_,_,_ = self.get_reference()

		# Observation
		_ = self.get_state()
		_ = self.get_state_norm()
		observation = self.get_obs()
		self.observation = []
		for _ in range(8):
			self.observation += observation
		print('... S T A R T ...')
		return self.observation

	def client_reset(self):
		self.client.reset()

	def step(self,action,action_rotor):
		done = False
		done_type = False

		# New State
		_ = self.get_state()
		_ = self.get_state_norm()
		# action = np.clip(action,-1.,1.)
		self.action = action

		# Rescale action
		# Synchronize the timestamp >> Use it while using controller
		time_now = (self.get_state()[-1] - self.initial_time)/1e9
		self.client.simPause(False)
		self.client.moveByRotorSpeedAsync(action_rotor[0],action_rotor[1],action_rotor[2],action_rotor[3],self.T).join()
		self.client.simPause(True)

		if self.is_controller == True:
			while abs(self.r_t[self.t_idx]-time_now) > 0.003*2:
				self.t_idx += 1
				if (len(self.r_t) - self.t_idx) <= 1:
					done_type = 1
					self.t_idx = 0
					print('End Type : Time Syncronization Failed',"\n")
					break

		# Find Orthogonal Point
		self.idx, self.dist = self.find_orth()
		self.idx += 15

		# Termination Check
		collision = self.client.simGetCollisionInfo()
		collision = collision.has_collided
		if collision == True:
			done_type = 2
			print('collision')
			done = True
		# if len(self.r_x) - self.t_idx < 50:
		# 	done_type = 3
		# 	print('idx out1 ', self.t_idx)
		# 	done = True
		if abs(self.phi_norm*180) > 89. or abs(self.theta_norm*180) > 89. or abs(self.err_psi*180) > 170.:
			done_type = 4
			print('tilted ')
			done = True
		if abs(self.err_x) > self.threshold1 or abs(self.err_y) > self.threshold1 or abs(self.err_z) > self.threshold1:
			done_type = 5
			print('out of track', self.err_x, self.err_y, self.err_z)
			done = True
		# if abs(self.x) > 15.:
		# 	done_type = 5.5
		# 	print('gone too far')
		# 	done = True
		if self.idx > len(self.r_x)-100:
			done_type = 6
			print('time out', self.idx)
			done = True
			self.observation = [0. for _ in range(self.num_states)]
			reward = 0.
		else: # If NOT Terminated
			self.calculate_error()			
			# New State
			_ = self.get_state()
			_ = self.get_state_norm()
			self.observation = self.observation[6::] + self.get_obs()
			reward = self.get_reward(done_type)
			self.prev_dist = self.dist
			self.prev_action = self.action
			self.t_idx += 1
			self.count += 1
		info = done_type
		return self.observation, reward, done, info

	def denormalizeAction(self,action):
		return (action+1.)/2.*(6396.667*2*np.pi/60.)*2.

	def convert2rotor(self,action):
		# 		+x
		# 		|		
		# 	f2	|  f3
		# -------------+y
		# 	f1	|  f4
		# 		|

		d1 = 0.2275*np.cos(45.*np.pi/180.)
		d2 = 0.2275*np.cos(45.*np.pi/180.)
		c = 0.1

		action = [(action[0]+1)/2*15.,action[1]*0.1,action[2]*0.1,action[3]*0.1]
		# action = [action[0]*100.,action[1]*100.,action[2]*100.,action[3]*100.]
		U = [action[0],action[1],action[2],action[3]]
		f1 = (U[0]/4. + U[1]/(4.*d2) - U[2]/(4.*d1) + U[3]/(4.*c))
		f2 = (U[0]/4. + U[1]/(4.*d2) + U[2]/(4.*d1) - U[3]/(4.*c))
		f3 = (U[0]/4. - U[1]/(4.*d2) + U[2]/(4.*d1) + U[3]/(4.*c))
		f4 = (U[0]/4. - U[1]/(4.*d2) - U[2]/(4.*d1) - U[3]/(4.*c))
		if f1 < 0:
			f1 = 0.
		if f2 < 0:
			f2 = 0.
		if f3 < 0:
			f3 = 0.
		if f4 < 0:
			f4 = 0.

		# Conver into rad/s
		# np.sqrt(f1/pow((0.2286),4)/1.225/0.109919) term is in rotation per second
		M1 = np.sqrt(f1/pow((0.2286),4)/1.225/0.109919)*2*np.pi 
		M2 = np.sqrt(f2/pow((0.2286),4)/1.225/0.109919)*2*np.pi
		M3 = np.sqrt(f3/pow((0.2286),4)/1.225/0.109919)*2*np.pi
		M4 = np.sqrt(f4/pow((0.2286),4)/1.225/0.109919)*2*np.pi

		# max_thrust = 4.17944
		# max_rotate = 6396.667
		# conversion = 2*np.pi/60. (from rotation/sec to rad/sec)

		rotor = [M1,M2,M3,M4]
		rotor_nan_check = np.isnan(rotor)
		if all(rotor_nan_check) == True:
			rotor = [0.,0.,0.,0]
		# rotor = np.clip(rotor,0.,6396.667*2*np.pi/60.)
		return rotor


