import math
import numpy as np
from numpy.linalg import multi_dot


from .Controller_R import RLS_C_R
from .Controller_T import RLS_C_T
from .Estimator_R import RLS_EHGO_R
from .Estimator_T import RLS_EHGO_T



class RLS_Controller:
	def __init__(self,initial_state,general_s, EHGO_R_s, EHGO_T_s, Controller_R_s, Controller_T_s):
		# Configure States

		self.C_R = RLS_C_R(general_s,Controller_R_s)
		self.C_T = RLS_C_T(general_s, Controller_T_s)
		self.E_R = RLS_EHGO_R(general_s, EHGO_R_s)
		self.E_T = RLS_EHGO_T(general_s, EHGO_T_s)

		self.linear_position = initial_state[0:3]
		self.linear_velocity = initial_state[3:6]
		self.angular_orientation = initial_state[6:9]
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

	def angular_orientation_conversion(self,roll,pitch,yaw,w):
		self.angular_orientation = self.quaternion_to_euler(roll,pitch,yaw,w)
		self.yaw_hist += [self.angular_orientation[2]]

		if self.yaw_hist[-1] - self.yaw_hist[-2] < -1.57:
			self.rotation += 1
		elif self.yaw_hist[-1] - self.yaw_hist[-2] > 1.57:
			self.rotation -= 1
		
		self.angular_orientation[2] = self.angular_orientation[2]+2*np.pi*self.rotation
		self.yaw_hist = [self.yaw_hist[-1]]
		return self.angular_orientation

	def organize_state(self,state,reference):
		# Configure States
		self.linear_position = state[0:3]
		self.linear_velocity = state[3:6]
		self.angular_orientation = self.angular_orientation_conversion(state[6],state[7],state[8],state[9])
		self.angular_velocity = state[10:13]
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

		self.EHGO_R_output = self.E_R.EHGO_R(T, self.EHGO_R_input)
		self.EHGO_T_output = self.E_T.EHGO_T(T, self.EHGO_T_input)

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
		psi2_r = self.angular_velocity[2]
		psi2_r = 0.
		U1_ = self.controller_T_output[2]
		U1f_ = self.controller_T_output[3]
		# Rotational Controller
		self.controller_R_input = [phi_v_, theta_v_, psi2_r] \
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
		rotor = rotor/(6396.667*2*np.pi/60.)*2.-1. # Normalize
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