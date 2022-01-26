from argparse import ArgumentParser
import numpy as np
import csv
import time
from matplotlib import pyplot as plt
import random
import math

class Env():
	def __init__(self, Quadrotor, num_states, num_actions):
		self.num_states = num_states
		self.num_actions = num_actions

		self.client = Quadrotor
		self.client.confirmConnection()
		self.client.enableApiControl(True)
		self.client.armDisarm(True)

		self.yaw_history = [0.]
		self.angular_orientation = []
		self.rotation = 0
		
		state = self.get_state()
		self.initial_position = [state[0], state[1], state[2]]
		self.initial_orientation = [state[6], state[7], state[8], state[9]]

		self.r_t = []
		self.r_x = []
		self.r_y = []
		self.r_z = []
		self.r_vx = []
		self.r_vy = []
		self.r_vz = []
		self.r_yaw = []

		self.elip_Rotaional=0.000002
		self.PInit = 0.0000001

		self.T = 0.0029
		self.g = 9.8
		self.m = 1.
		self.filter_value = 0.1
		# mass = 1.
		# motor_assembly_wight = 0.055
		# box_mass = 1 - 0.055*4 = 0.78
		# body_box.x = 0.18
		# body_box.y = 0.11
		# body_box.z = 0.04
		# rotor_z = 0.025
		self.Ixx = 0.78/12*(0.11*0.11+0.04*0.04) + 4*0.055*(0.025*0.025+0.275*0.70711*0.275*0.70711)
		self.Iyy = 0.78/12*(0.18*0.18+0.04*0.04) + 4*0.055*(0.025*0.025+0.275*0.70711*0.275*0.70711)
		self.Izz = 0.78/12*(0.18*0.18+0.11*0.11) + 4*0.055*(0.275*0.275)

		# Rotational EHGO
		self.epsilonR = 0.007
		self.r1 = 5.
		self.r2 = 1.
		self.r3 = 1.

		# # Translational EHGO
		# epsilonT = 0.05
		# a10 = 3.15
		# a11 = 2.95
		# a12 = 0.8

		# b10 = 2.1
		# b11 = 0.2675
		# b12 = 0.0263

		# Translational EHGO
		self.epsilonT = 0.06
		self.a10=3.15
		self.a11=2.95
		self.a12=0.8
		self.b10=2.1
		self.b11=0.2675
		self.b12=0.0263

		# Translational Control
		# kx1 = 2.8
		# kx2 = kx1*0.95

		## Change X Val kx up kx down
		# kx1 = 8.
		# kx2 = kx1*3.5

		# ky1 = 8.
		# ky2 = ky1*2.8

		# kz1 = 8.
		# kz2 = kz1*3.5

		self.kx1 = 4.
		self.kx2 = 3.7
		self.ky1 = 4.
		self.ky2 = 3.7
		self.kz1 = 10.
		self.kz2 = self.kz1*2.71
		# ep = 1.8
		# kpr = 50. / pow(ep,2)
		# kvr = 10. / ep
		self.ep = 0.1
		self.kpr = 2./pow(self.ep,2)
		self.kvr = 4./self.ep

		# ep = 0.1
		# kpr = 2./pow(ep,2)
		# kvr = 4./ep


	def get_settings(self):
		general_settings = [self.T, self.g, self.m, self.Ixx, self.Iyy, self.Izz, self.filter_value]
		EHGO_R_settings = [self.epsilonR, self.r1, self.r2, self.r3]
		EHGO_T_settings = [self.epsilonT, self.a10, self.a11, self.a12, self.b10, self.b11, self.b12]
		Controller_R_settings = [self.elip_Rotaional, self.kpr, self.kvr]
		Controller_T_settings = [self.kx1, self.kx2, self.ky1, self.ky2, self.kz1, self.kz2]
		return general_settings, EHGO_R_settings, EHGO_T_settings, Controller_R_settings, Controller_T_settings

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

			0., 0., 0.
			]
		return Initial_state

	def quaternion_to_euler(self, x, y, z, w):
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll = math.atan2(t0, t1)
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch = math.asin(t2)
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw = math.atan2(t3, t4)
		return [roll, pitch, yaw]

	def orientation_conversion(self,roll,pitch,yaw,w):
		self.angular_orientation = self.quaternion_to_euler(roll,pitch,yaw,w)
		self.yaw_history = self.yaw_history + [self.angular_orientation[2]]

		if self.yaw_history[-1] - self.yaw_history[-2] < -1.57:
			self.rotation += 1
		elif self.yaw_history[-1] - self.yaw_history[-2] > 1.57:
			self.rotation -= 1
		
		self.angular_orientation[2] = self.angular_orientation[2]+2*np.pi*self.rotation
		self.yaw_history = [self.yaw_history[-1]]

		return self.angular_orientation

	def get_state(self):
		state = self.client.getMultirotorState()
		linear_position = state.kinematics_estimated.position
		x = linear_position.x_val 
		y = linear_position.y_val
		z = linear_position.z_val

		linear_velocity = state.kinematics_estimated.linear_velocity
		vx = linear_velocity.x_val
		vy = linear_velocity.y_val
		vz = linear_velocity.z_val

		angular_angle = state.kinematics_estimated.orientation
		phi = angular_angle.x_val
		theta = angular_angle.y_val
		psi = angular_angle.z_val
		q = angular_angle.w_val
		angle = self.orientation_conversion(phi, theta, psi, q)
		phi = angle[0]
		theta = angle[1]
		psi = angle[2]

		angular_velocity = state.kinematics_estimated.angular_velocity
		vphi = angular_velocity.x_val
		vtheta = angular_velocity.y_val
		vpsi = angular_velocity.z_val

		time_now = state.timestamp
		t = time_now

		# linear_acceleration = state.kinematics_estimated.linear_acceleration
		# angular_acceleration = state.kinematics_estimated.angular_acceleration

		state_current = [
						 x, y, z, vx, vy, vz,
						 phi, theta, psi, vphi, vtheta, vpsi,
						 t
						]

		return state_current

	def get_raw_state(self):
		state = self.client.getMultirotorState()
		linear_position = state.kinematics_estimated.position
		x = linear_position.x_val 
		y = linear_position.y_val
		z = linear_position.z_val

		linear_velocity = state.kinematics_estimated.linear_velocity
		vx = linear_velocity.x_val
		vy = linear_velocity.y_val
		vz = linear_velocity.z_val

		angular_angle = state.kinematics_estimated.orientation
		phi = angular_angle.x_val
		theta = angular_angle.y_val
		psi = angular_angle.z_val
		q = angular_angle.w_val

		angular_velocity = state.kinematics_estimated.angular_velocity
		vphi = angular_velocity.x_val
		vtheta = angular_velocity.y_val
		vpsi = angular_velocity.z_val

		time_now = state.timestamp
		t = time_now

		# linear_acceleration = state.kinematics_estimated.linear_acceleration
		# angular_acceleration = state.kinematics_estimated.angular_acceleration

		state_current = [
						 x, y, z, vx, vy, vz,
						 phi, theta, psi, q, vphi, vtheta, vpsi,
						 t
						]

		return state_current

	def get_reference(self):
		with open('C:/Users/lee/Desktop/Projects/Quadrotor/airsim/DDPG/airsim_env/Reference_sampled_not_extended_0.003.csv', 'r') as csvFile:
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

		return self.r_t, self.r_x, self.r_y, self.r_z, self.r_vx, self.r_vy, self.r_vz, self.r_yaw

	
	def take_off(self,t_idx):
		print('Take Off')
		self.current_state = self.get_state()
		initX = self.current_state[0]
		initY = self.current_state[1]
		initZ = -1.

		self.client.simPause(False)
		self.client.moveToPositionAsync(initX, initY, initZ, 1.).join()
		# mode = random.randrange(2)
		# if mode == 1:
		# 	self.client.moveByVelocityAsync(random.uniform(-0.5,0.5), random.uniform(-0.5,0.5), random.uniform(-0.5,0.5), 0.8).join()
		# else:
		self.client.moveByAngleZAsync(random.uniform(-0.1,0.1), random.uniform(-0.1,0.1), initZ +  random.uniform(-0.1,0.1), random.uniform(-0.1,0.1), 0.8).join()
		self.client.simPause(True)
		time.sleep(0.1)

		err_x = self.r_x[t_idx] - self.current_state[0]
		err_y = self.r_y[t_idx] - self.current_state[1]
		err_z = self.r_z[t_idx] - self.current_state[2]
		err_vx = self.r_vx[t_idx] - self.current_state[3]
		err_vy = self.r_vy[t_idx] - self.current_state[4]
		err_vz = self.r_vz[t_idx] - self.current_state[5]
		phi = self.current_state[6]/np.pi
		theta = self.current_state[7]/np.pi
		psi = self.current_state[8]/np.pi
		vphi = self.current_state[9]/np.pi
		vtheta = self.current_state[10]/np.pi
		vpsi = self.current_state[11]/np.pi

		observation = [err_x, err_y, err_z,err_vx, err_vy, err_vz, phi, theta, psi, vphi, vtheta, vpsi]

		return observation

	def get_reward(self,t_idx):
		self.current_state = self.get_state()
		err_x = abs(self.r_x[t_idx] - self.current_state[0])
		err_y = abs(self.r_y[t_idx] - self.current_state[1])
		err_z = abs(self.r_z[t_idx] - self.current_state[2])
		# err_vx = abs(self.r_vx[t_idx] - self.current_state[3])
		# err_vy = abs(self.r_vy[t_idx] - self.current_state[4])
		# err_vz = abs(self.r_vz[t_idx] - self.current_state[5])

		# reward = - (err_x + err_y + err_z + err_vx + err_vy + err_vz) / 1000.
		reward = - (err_x + err_y + err_z) / 10.

		return reward


	def reset(self):
		self.client.simPause(False)
		self.client.moveByRotorSpeedAsync(0.,0.,0.,0.,1.5).join()
		self.client.simPause(True)

		pose = self.client.simGetVehiclePose()
		pose.position.x_val = self.initial_position[0]
		pose.position.y_val = self.initial_position[1]
		pose.position.z_val = self.initial_position[2]
		pose.orientation.x_val = self.initial_orientation[0]
		pose.orientation.y_val = self.initial_orientation[1]
		pose.orientation.z_val = self.initial_orientation[2]
		pose.orientation.w_val = self.initial_orientation[3]

		self.client.simSetVehiclePose(pose, True)

		self.client.simPause(False)
		self.client.moveByRotorSpeedAsync(0.,0.,0.,0.,1.).join()
		time.sleep(2.)
		self.client.simPause(True)


		

	def step(self,action,t_idx):
		done = False
		action = (np.array(action)+1.)/2.*(6396.667*2*np.pi/60.)
		action = np.clip(action,0.,6396.667*2*np.pi/60.)
		self.client.simPause(False)
		self.client.moveByRotorSpeedAsync(action[0],action[1],action[2],action[3],0.001).join()
		self.client.simPause(True)
		reward = self.get_reward(t_idx)

		err_x = self.r_x[t_idx] - self.current_state[0]
		err_y = self.r_y[t_idx] - self.current_state[1]
		err_z = self.r_z[t_idx] - self.current_state[2]
		err_vx = self.r_vx[t_idx] - self.current_state[3]
		err_vy = self.r_vy[t_idx] - self.current_state[4]
		err_vz = self.r_vz[t_idx] - self.current_state[5]
		phi = self.current_state[6]/np.pi
		theta = self.current_state[7]/np.pi
		psi = self.current_state[8]/np.pi
		vphi = self.current_state[9]/np.pi
		vtheta = self.current_state[10]/np.pi
		vpsi = self.current_state[11]/np.pi
		observation = [err_x, err_y, err_z, err_vx, err_vy, err_vz, phi, theta, psi, vphi, vtheta, vpsi]

		collision = self.client.simGetCollisionInfo()
		collision = collision.has_collided
		if collision == True:
			done = True
			reward = -50.
		if len(self.r_x) - t_idx < 1000:
			done = True
		
		return observation, reward, done



