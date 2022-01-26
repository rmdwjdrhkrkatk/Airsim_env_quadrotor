import math
import numpy as np
from numpy.linalg import multi_dot

from .Controller_R import RLS_C_R
from .Controller_T import RLS_C_T
from .Estimator_R import RLS_EHGO_R
from .Estimator_T import RLS_EHGO_T

class RLS_Controller:
	def __init__(self):
		self.elip_Rotaional=0.0000015
		self.PInit = 0.0000001

		self.T = 0.0015
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

		# Translational EHGO
		self.epsilonT = 0.06
		self.a10=3.15
		self.a11=2.95
		self.a12=0.8
		self.b10=2.1
		self.b11=0.2675
		self.b12=0.0263

		# Translational Control
		self.kx1 = 4.
		self.kx2 = self.kx1 * 1.25
		self.ky1 = 4.
		self.ky2 = self.kx2 * 1.25
		self.kz1 = 12.
		self.kz2 = self.kz1 * 2.71

		self.ep = 0.15
		self.kpr = 2./pow(self.ep,2)
		self.kvr = 4./self.ep
		
		# Configure States
		self.C_R = 0.
		self.C_T = 0.
		self.E_R = 0.
		self.E_T = 0.

		self.linear_position = []
		self.linear_velocity = []
		self.angular_orientation = []
		self.initial_yaw = []
		self.angular_velocity = []

		self.U = 0.
		self.Uf = 0.

		self.ob_R = 0.
		self.ob_R2 = 0.
		self.ob_T = 0.
		self.ob_T2 = 0.

		self.sig_R = 0.
		self.sig_T = 0.

		self.reference_T = 0.
		self.reference_T2 = 0.
		self.reference_T3 = 0.

		self.p = 0.
		self.angular_orientation_v = 0.

		# Inputs and Outputs for EHGOs
		self.EHGO_R_input = []
		self.EHGO_R_output = []

		self.EHGO_T_input = []
		self.EHGO_T_output = []

		# Inputs and Outputs for Controllers
		self.controller_R_input = []
		self.controller_R_output = []

		self.controller_T_input = []
		self.controller_T_output = []

		# Variables Settings
		self.controller_R_settings = 0.
		self.controller_T_settings = 0.

		self.EHGO_T_settings = 0.
		self.EHGO_R_settings = 0.

		# Yaw Computation
		self.rotation = 0
		self.yaw_hist = [0.]

	def get_settings(self):
		general_settings = [self.T, self.g, self.m, self.Ixx, self.Iyy, self.Izz, self.filter_value]
		EHGO_R_settings = [self.epsilonR, self.r1, self.r2, self.r3]
		EHGO_T_settings = [self.epsilonT, self.a10, self.a11, self.a12, self.b10, self.b11, self.b12]
		Controller_R_settings = [self.elip_Rotaional, self.kpr, self.kvr]
		Controller_T_settings = [self.kx1, self.kx2, self.ky1, self.ky2, self.kz1, self.kz2]
		return general_settings, EHGO_R_settings, EHGO_T_settings, Controller_R_settings, Controller_T_settings

	def reset(self,initial_state,general_s, EHGO_R_s, EHGO_T_s, Controller_R_s, Controller_T_s):
		# Configure States
		self.C_R = RLS_C_R(general_s,Controller_R_s)
		self.C_T = RLS_C_T(general_s, Controller_T_s)
		self.E_R = RLS_EHGO_R(general_s, EHGO_R_s)
		self.E_T = RLS_EHGO_T(general_s, EHGO_T_s)

		self.linear_position = initial_state[0:3]
		self.linear_velocity = initial_state[3:6]
		self.angular_orientation = initial_state[6:9]
		self.initial_yaw = self.angular_orientation[2]
		self.angular_velocity = initial_state[9:12]

		self.U = initial_state[12:16]
		self.Uf = initial_state[16:20]

		self.ob_R = initial_state[20:23]
		self.ob_R2 = initial_state[23:26]
		self.ob_T = initial_state[26:29]
		self.ob_T2 = initial_state[29:32]

		self.sig_R = initial_state[32:35]
		self.sig_T = initial_state[35:38]   

		self.reference_T = initial_state[38:41]
		self.reference_T2 = initial_state[41:44]
		self.reference_T3 = initial_state[44:47]     

		self.p = initial_state[47:56]
		self.angular_orientation_v = initial_state[56:59]

		# Inputs and Outputs for EHGOs
		self.EHGO_R_input = []
		self.EHGO_R_output = []

		self.EHGO_T_input = []
		self.EHGO_T_output = []

		# Inputs and Outputs for Controllers
		self.controller_R_input = []
		self.controller_R_output = []

		self.controller_T_input = []
		self.controller_T_output = []

		# Variables Settings
		self.controller_R_settings = general_s + Controller_R_s
		self.controller_T_settings = general_s + Controller_T_s

		self.EHGO_T_settings = general_s + EHGO_T_s
		self.EHGO_R_settings = general_s + EHGO_R_s

		# Yaw Computation
		self.rotation = 0
		self.yaw_hist = [0.]
		
	def project(self, action, action_ref, range_val=0.):
		r = float(range_val)
		action_min = np.clip(np.array([action_ref[0]-r, action_ref[1]-r, action_ref[2]-r, action_ref[3]-r]),-1.,1.)
		action_max = np.clip(np.array([action_ref[0]+r, action_ref[1]+r, action_ref[2]+r, action_ref[3]+r]),-1.,1.)
		action_range = list(zip(action_min, action_max))
		action_range = action_range[:]
		action_range = list(action_range)

		for i in range(4):
			if action[i] <= action_range[i][0]:
				action[i] = action_range[i][0]
			elif action[i] >= action_range[i][1]:
				action[i] = action_range[i][1]

		action = np.array(action,dtype=float)
		return action

	# def quaternion_to_euler(self, x, y, z, w):
	# 	t0 = +2.0 * (w * x + y * z)
	# 	t1 = +1.0 - 2.0 * (x * x + y * y)
	# 	roll = math.atan2(t0, t1)
	# 	t2 = +2.0 * (w * y - z * x)
	# 	t2 = +1.0 if t2 > +1.0 else t2
	# 	t2 = -1.0 if t2 < -1.0 else t2
	# 	pitch = math.asin(t2)
	# 	t3 = +2.0 * (w * z + x * y)
	# 	t4 = +1.0 - 2.0 * (y * y + z * z)
	# 	yaw = math.atan2(t3, t4)
	# 	return [roll, pitch, yaw]

	# def angular_orientation_conversion(self,roll,pitch,yaw,w):
	# 	self.angular_orientation = self.quaternion_to_euler(roll,pitch,yaw,w)
	# 	self.yaw_hist += [self.angular_orientation[2]]

	# 	if self.yaw_hist[-1] - self.yaw_hist[-2] < -1.57:
	# 		self.rotation += 1
	# 	elif self.yaw_hist[-1] - self.yaw_hist[-2] > 1.57:
	# 		self.rotation -= 1
		
	# 	self.angular_orientation[2] = self.angular_orientation[2]+2*np.pi*self.rotation
	# 	self.yaw_hist = [self.yaw_hist[-1]]

	# 	return self.angular_orientation

	def organize_state(self,state,reference):
		# Configure States
		self.linear_position = state[0:3]
		self.linear_velocity = state[3:6]
		self.angular_orientation = state[6:9]
		self.angular_velocity = state[9:12]
		self.reference_T = reference[0:3]
		self.reference_T2 = reference[3:6]
		# self.reference_T3 = reference[6:9]
		self.reference_T3 = [0., 0., 0.]


	def EHGO_Integrated(self,T):
		# Estimation
		self.EHGO_R_input = [self.Uf[1], self.Uf[2], self.Uf[3]] \
						+ self.angular_orientation + self.angular_velocity \
						+ self.ob_R2 + self.sig_R + [T]
		self.EHGO_T_input = [self.Uf[0]] \
						+ self.angular_orientation + self.linear_velocity \
						+ self.ob_T2 + self.sig_T + [T]

		self.EHGO_R_output = self.E_R.EHGO_R(self.EHGO_R_input)
		self.EHGO_T_output = self.E_T.EHGO_T(self.EHGO_T_input)

		# Update EHGOs states
		self.ob_R2 = self.EHGO_R_output[0:3]
		self.sig_R = self.EHGO_R_output[3:6]
		self.ob_T2 = self.EHGO_T_output[0:3]
		self.sig_T = self.EHGO_T_output[3:6]
		# return self.EHGO_R_output, self.EHGO_T_output
	 

	def Controller_Integrated(self,T):
		# 		+x
		# 		|		
		# 		|
		# -------------+y
		# 		|
		# 		|
		# Translational Controller
		self.controller_T_input = self.reference_T + self.reference_T2 + self.reference_T3 \
								+ self.linear_position + self.linear_velocity + self.sig_T \
								+ self.angular_orientation + [self.Uf[0]]  + [T]
		self.controller_T_output = self.C_T.controller_translation(T, self.controller_T_input)

		phi_v_ = self.controller_T_output[0]
		theta_v_ = self.controller_T_output[1]
		psi_r = self.angular_orientation[2]
		psi_r = 0.
		# psi2_r = self.angular_velocity[2]
		U1_ = self.controller_T_output[2]
		U1f_ = self.controller_T_output[3]
		# Rotational Controller
		self.controller_R_input = [phi_v_, theta_v_, psi_r] \
								+ self.angular_orientation + self.angular_velocity \
								+ self.sig_R \
								+ [self.Uf[1], self.Uf[2], self.Uf[3]] \
								+ self.p  + [T]
		self.controller_R_output = self.C_R.controller_rotation(T, self.controller_R_input)
		# Update States
		self.U = [self.controller_T_output[2]] + self.controller_R_output[3:6]
		self.Uf = [self.controller_T_output[3]] + self.controller_R_output[0:3]

		self.angular_orientation_v = self.controller_T_output[0:2] + [self.angular_orientation[2]]
		self.p = self.controller_R_output[6:15]

		# return self.U
	
	def return_r_input(self):
		return self.controller_R_input

	def Mixer(self):

		# 		+x
		# 		|		
		# 	f2	|  f3
		# -------------+y
		# 	f1	|  f4
		# 		|

		d1 = 0.2275*np.cos(45.*np.pi/180.)
		d2 = 0.2275*np.cos(45.*np.pi/180.)
		c = 0.1

		# self.U = [0.,0.,0.,0.7]
		f1 = (self.U[0]/4 + self.U[1]/(4*d2) - self.U[2]/(4*d1) + self.U[3]/(4*c))#+0.149#*1000/9.81
		f2 = (self.U[0]/4 + self.U[1]/(4*d2) + self.U[2]/(4*d1) - self.U[3]/(4*c))#+0.149#*1000/9.81
		f3 = (self.U[0]/4 - self.U[1]/(4*d2) + self.U[2]/(4*d1) + self.U[3]/(4*c))#+0.149#*1000/9.81
		f4 = (self.U[0]/4 - self.U[1]/(4*d2) - self.U[2]/(4*d1) - self.U[3]/(4*c))#+0.149#*1000/9.81


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

		# M1 = f1/max_thrust * max_rotate * conversion
		# M2 = f2/max_thrust * max_rotate * conversion
		# M3 = f3/max_thrust * max_rotate * conversion
		# M4 = f4/max_thrust * max_rotate * conversion

		rotor = [M1,M2,M3,M4]
		rotor_nan_check = np.isnan(rotor)
		if all(rotor_nan_check) == True:
			rotor = [0.,0.,0.,0]
			# print('NaN')

		# min 365
		# 6396.667 is maximum rotational velocity in rotation per second 
		rotor = np.clip(rotor,0.,6396.667*2*np.pi/60.)
		# rotor = rotor/(6396.667*2*np.pi/60.)*2.-1. #Normalize

		# rotor = [self.U[0],self.U[1],self.U[2],self.U[3]]
		return rotor

	def return_variables(self):
		variables = \
		self.linear_position \
		+ self.linear_velocity \
		+ self.angular_orientation \
		+ self.angular_velocity \
		+ self.U \
		+ self.Uf \
		+ self.ob_R \
		+ self.ob_R2 \
		+ self.ob_T \
		+ self.ob_T2 \
		+ self.sig_R \
		+ self.sig_T \
		+ self.reference_T \
		+ self.reference_T2 \
		+ self.reference_T3 \
		+ self.p \
		+ self.angular_orientation_v \
		+ self.EHGO_R_output \
		+ self.EHGO_T_output \
		+ self.controller_R_output \
		+ self.controller_T_output

		# 0 self.linear_position \
		# 3 + self.linear_velocity \
		# 6 + self.angular_orientation \
		# 9 + self.angular_velocity \
		# 12 + self.U \
		# 16 + self.Uf \
		# 20 + self.ob_R \
		# 23 + self.ob_R2 \
		# 26 + self.ob_T \
		# 29 + self.ob_T2 \
		# 32 + self.sig_R \
		# 35 + self.sig_T \
		# 38 + self.reference_T \
		# 41 + self.reference_T2 \
		# 44 + self.reference_T3 \
		# 47 + self.p \
		# 56 + self.angular_orientation_v \
		# 59 + self.EHGO_R_output \
		# 65 + self.EHGO_T_output \
		# 71 + self.controller_R_output \
		# 86 + self.controller_T_output 90

		return variables