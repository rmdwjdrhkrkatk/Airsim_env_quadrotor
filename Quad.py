import time
import sys
import numpy as np
from numpy.linalg import multi_dot

import rospy
import std_msgs.msg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mav_msgs.msg import CommandMotorSpeed

from Controller_Combined import RLS_Controller

PInit = 10.
Initial_state = \
	[
	0., 0., 0., \
	0., 0., 0., \
	0., 0., 0., \
	0., 0., 0., \

	0., 0., 0., 0., \
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

	PInit, 0., 0., \
	0., PInit, 0., \
	0., 0., PInit, \

	0., 0., 0.
	]



RLS = RLS_Controller(Initial_state)

class Quad_class:
	def __init__(self,motor): 
		# Define Publisher
		

		# Start Subscribing at the begining of the Class
		

		# Define array
		self.linear_position = []
		self.linear_velocity = []
		self.angular_orientation = [] 
		self.angular_velocity = []
		self.state = []
		self.reference = []

		# Set variables
		self.t = 0.
		self.k = 0
		self.flight_mode = 'Auto'
		self.rotors = []

		# Plot
		self.data = []
		self.data_selected = []
		self.data_num = 10
		self.x_vec = np.linspace(0,1,25000)[0:-1]
		self.y_vec = np.zeros(len(self.x_vec))
		self.line = []
        self.motor_pub = motor


	def odo_callback(self, msg):
		# Subscribe
		self.linear_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
		self.linear_velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
		self.angular_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
		self.angular_velocity = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
		self.state = self.linear_position + self.linear_velocity + self.angular_orientation + self.angular_velocity

		# Make Reference
		self.reference = [5*np.sin(self.t), 5*np.cos(self.t), 5*np.sin(self.t),\
						5*np.cos(self.t), -5*np.sin(self.t), 5*np.cos(self.t),\
						-5*np.sin(self.t), -5*np.cos(self.t), -5*np.sin(self.t)]

		# Control
		RLS.organize_state(self.state,self.reference)
		RLS.EHGO_Integrated()
		RLS.Controller_Integrated()
		self.rotor = RLS.Mixer()

		# Publish
		rotor_msg = CommandMotorSpeed()
		rotor_msg.header.stamp = rospy.Time.now()
		rotor_msg.motor_speed = [self.rotor[2],self.rotor[1],self.rotor[3],self.rotor[0]]
		# rotor_msg.motor_speed = [11,11,0,11]
		self.motor_pub.publish(rotor_msg)

		
		# Plot Data
		self.data = RLS.return_variables()
		self.data_selected = self.data[self.data_num]
		self.y_vec[-1] = self.data_selected
		self.line = self.live_plotter()
		self.y_vec = np.append(self.y_vec[1:],0.0)

		self.time += 0.004
		

	def live_plotter(self,identifier='',pause_time=0.1):
		if self.line == []:
			plt.ion()
			fig = plt.figure(figsize=(13,8))
			ax = fig.add_subplot(111)

			self.line, = ax.plot(self.x_vec,self.y_vec,'-o',alpha=0.8)
			plt.ylable('Y')
			plt.xlable('Time')
			plt.show()
		
		self.line.set_ydata(self.y_vec)

		if np.min(self.y_vec)<=self.line.axes.get_ylim()[0] or np.max(self.y_vec)>=self.line.axes.get_ylim()[1]:
			plt.ylim([np.min(self.y_vec)-np.std(self.y_vec),np.max(self.y_vec)+np.std(self.y_vec)])

		plt.pause(pause_time)