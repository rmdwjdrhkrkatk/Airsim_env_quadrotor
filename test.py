import setup_path 
import airsim

from argparse import ArgumentParser
import numpy as np
import math

import csv
import time
from matplotlib import pyplot as plt




###################### Take off ######################
initX = 0.
initY = 0.
initZ = -5.1

# connect to the AirSim simulator 
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.moveToPositionAsync(initX, initY, initZ, 1.6).join()
print('arrived')
# time.sleep(1.)
#######################################################


def quaternion_to_euler(x, y, z, w):

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
    return [yaw, pitch, roll]
############################################################################################################################
x = []
log = []
elapsed_time = 0.

state_airsim = client.getMultirotorState()
time_init = state_airsim.timestamp

time_now = 0.
toc = 0.

t = 0. 
d1 = 0.2275*np.cos(45.*np.pi/180.)
d2 = 0.2275*np.cos(45.*np.pi/180.)
c = 0.1

U = [0,0,0,0.7]
print('go')
f1 = (U[0]/4 + U[1]/(4*d2) - U[2]/(4*d1))# - U[3]/(4*c))#+0.149#*1000/9.81
f2 = (U[0]/4 + U[1]/(4*d2) + U[2]/(4*d1))# + U[3]/(4*c))#+0.149#*1000/9.81
f3 = (U[0]/4 - U[1]/(4*d2) + U[2]/(4*d1))# - U[3]/(4*c))#+0.149#*1000/9.81
f4 = (U[0]/4 - U[1]/(4*d2) - U[2]/(4*d1))# + U[3]/(4*c))#+0.149#*1000/9.81


# Conver into rad/s
# np.sqrt(f1/pow((0.2286),4)/1.225/0.109919) term is in rotation per second
M1 = np.sqrt(f1/pow((0.2286),4)/1.225/0.109919)*2*np.pi 
M2 = np.sqrt(f2/pow((0.2286),4)/1.225/0.109919)*2*np.pi
M3 = np.sqrt(f3/pow((0.2286),4)/1.225/0.109919)*2*np.pi
M4 = np.sqrt(f4/pow((0.2286),4)/1.225/0.109919)*2*np.pi
rotor = [M1,M2,M3,M4]
rotors = np.clip(rotor,0.,6396.667*2*np.pi/60.)
y = []
x = []

# client.simPause(False)
client.moveByAngleZAsync(0.,0,-5.1,3.1416*1.5,3).join()
# client.rotateByYawRateAsync(10, 100.)
# rotors = [500,0,500,0]
# client.moveByRotorSpeedAsync(rotors[0],rotors[1],rotors[2],rotors[3],0.001)

# client.simPause(True)
state_airsim = client.getMultirotorState()
angular_velocity = state_airsim.kinematics_estimated.angular_velocity
angular_angle = state_airsim.kinematics_estimated.orientation
linear_velocity = state_airsim.kinematics_estimated.linear_velocity
linear_position = state_airsim.kinematics_estimated.position
time_stamp = state_airsim.timestamp
time_now = (time_stamp - time_init)/1000000000.

yaw, pitch, roll = quaternion_to_euler(angular_angle.x_val,angular_angle.y_val,angular_angle.z_val,angular_angle.w_val)
print(yaw)
# pitch > arccos