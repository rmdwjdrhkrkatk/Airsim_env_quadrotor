import numpy as np
from numpy.linalg import multi_dot

import matplotlib.pyplot as plt



### Success List
## 1
elip_Rotaional=0.000003
PInit = 0.00000005
dtf = 0.55

# # 2
# elip_Rotaional=0.000005
# PInit = 0.0000004
# dtf = 0.55

# ##3
# elip_Rotaional=0.00001
# PInit = 0.000001
# dtf = 0.55

# 4   
# elip_Rotaional=0.00004
# PInit = 0.00001
# dtf = 0.55

## 5
# elip_Rotaional=0.0005
# PInit = 0.0008
# dtf = 0.55  

## 6
# elip_Rotaional=0.0006
# PInit = 0.00085
# dtf = 0.51 

# 7
# elip_Rotaional=0.0006
# PInit = 0.00088
# dtf = 0.51 

# elip_Rotaional=0.000001
# PInit = 0.1



# 1 x y z positions 3
# 4 vx vy vz velocities6
# 7 inputs Tm phi theta 9
# 10 reference xr yr zr positions
# 13 reference vxr vyr vzr velocities 15
# 16 angles 18
# 19 angular velocities 21
# 22 u2 u3 u4 24
# 25 Estimates phi [angle angular_velocity sigma] 27
# 28 Estimates theta psi [angle angular_velocity sigma] 30
# 31 Estimates  psi [angle angular_velocity sigma] 33
# 34 Estimates for x direction
# 37 Estimates for y direction
# 40 Estimates for z directio
# P 43 row1
# P 43 row1
# P 46 row2
# P 49 row 3

x=[1., 1., 1., \
   0.2, -0.1, 0.1, \
   0.5, 0.5, 0.5, \
   0.1, 0., 0., \
   1., 0., 1., \
   0., 0., 0., \
   0., 0., 0., \
   3., 0., 0., \
   1., 0.1, 0.1, \
   1., 1., 1., \
   1., 0., 0., \
   0., 0., 0., \
   0., 0., 0., \
   0., 0., 0., \
   PInit, 0., 0., \
   0., PInit, 0., \
   0., 0., PInit, \
   0.,0.,0.,0.
   ]


# x=[0., 0., 5., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    0., 0., 0., \
#    PInit, 0., 0., \
#    0., PInit, 0., \
#    0., 0., PInit, \
#    0.,0.,0.,0.
#    ]


Hz = 250
dt = 1./Hz
 
# dtf = 0.46

# a = dt/(dt+dtf)
a = 0.1

t = 0
timespan = 10

X = np.zeros([timespan*Hz,55])
T = np.arange(0,timespan,dt)

m=0.68
# ixx = 0.02024*0.46
# iyy = 0.01359*0.46
# izz = 0.03626*0.46
ixx = 0.007
iyy = 0.007
izz = 0.012

g=9.8

# Parameters for Dynamic Inversion
# elip_Rotaional=0.84








# Parameters for EHGOs
# Rotational EHGO
elipHr=0.032
# alpha11=6
# alpha12=6
# alpha13=2
# alpha21=6
# alpha22=6
# alpha23=2
# alpha31=6
# alpha32=6
# alpha33=2
# elipH=0.05
alpha11=1.5
alpha12=0.74
alpha13=0.12
alpha21=alpha11
alpha22=alpha12
alpha23=alpha13
alpha31=alpha11
alpha32=alpha12
alpha33=alpha13
# Translational EHGO
elipHt = 0.021

alpha11p=3.15
alpha21p=2.95
alpha31p=0.8
beta11p=2.1
beta21p=.2675
beta31p=0.0263


# Parameters for T and R Dynamics
kp=12.
kv=8.
kpz = 6
kvz = kpz*2.71

# elipr=0.1
# kpr=50/elipr**2
# kvr=10/elipr

ep = 0.1
kpr = 2./pow(ep,2)
kvr = 4./ep


for i in range(timespan*Hz):
# for i in range(1000):
   for j in range(55):
      X[i,j]=x[j]
      T[i]=t

   t = t + dt
   ####The State matrix####
#     dx = zeros[51,1]  # a column vector
   ######Parameters for Helicopter Controller####
   #Tt=8
   #a1s=[40*pi/180]*np.sin(T)
   #b1s=[40*pi/180]*np.sin[t+pi/8]
   
   ####################################
   ####################################

   # ##########################################################################
   pio = x[15]
   thetao = x[16]
   psio = x[17]
   dpio = x[18]
   dthetao = x[19]
   dpsio = x[20]

   x_1 = x[0]
   y_1 = x[1]
   z_1 = x[2]

   x_2 = x[3]
   y_2 = x[4]
   z_2 = x[5]


   AngPs = 0.01*np.sin(t*0.01)
   psi_r = AngPs

   # Acc Reference
   r1 = - 5*np.sin(t)
   r2 = - 5*np.cos(t)
   r3 = - 5*np.sin(t)


   sig_x = 3*np.sin(t)
   sig_y = 3*np.sin(t)
   sig_z = 3*np.sin(t)

   f_x = -kp*(x_1-5*np.sin(t)) - kv*(x_2-5*np.cos(t)) + r1 - sig_x
   f_y = -kp*(y_1-5*np.cos(t)) - kv*(y_2+5*np.sin(t)) + r2 - sig_y
   f_z = -kpz*(z_1-5*np.sin(t)) - kvz*(z_2-5*np.cos(t)) + r3 - sig_z
   # f_z = -kpz*(z_1-5.) - kvz*(z_2-0.) + r3 - sig_z


   ### DEFINE U1
   # u1 = - [m*[f_z - g)) / [np.cos[thetao)*np.cos(pio))

   ### DEFINE THETA_V
   y_th = f_y*np.sin(psi_r) + f_x*np.cos(psi_r)
   x_th = f_z-g
   theta_v = np.arctan(y_th/x_th)

   ### DEFINE PI_V
   y_pi = f_x*np.cos(theta_v)*np.sin(psi_r)-f_y*np.cos(theta_v)*np.cos(psi_r)
   x_pi = f_z-g
   pi_v = np.arctan(y_pi/x_pi)

   # u1 = - (m*(f_z - g)) / (np.cos(thetao)*np.cos(pio))
   u1 = - (m*(f_z - g)) / (np.cos(theta_v)*np.cos(pi_v))
   
#     u1 = beta*u1 + [1-beta]*u1

   # u1=x[7]#Tm
   # u2=x[8]#phi
   # u3=x[9]#theta
   #AngPs=0.1*np.sin(t)


   ###########################################################################
   ###########################################################################
   ###########################################################################
   ###########################################################################
   # # Tto=x[22]
   u2 = x[21]
   u3 = x[22]
   u4 = x[23]


   p11=x[42]
   p12=x[43]
   p13=x[44]
   p21=x[45]
   p22=x[46]
   p23=x[47]
   p31=x[48]
   p32=x[49]
   p33=x[50]

   PMat = np.array([[p11, p12, p13], \
         [p21, p22, p23], \
         [p31, p32, p33] \
         ])
   # a1so=x[23]
   # b1so=x[24]
   # a1s=a1so
   # b1s=b1so
   # a1s=min[abs[a1so],50*pi/180]*np.sign[a1so]#a1so
   # b1s=min[abs[b1so],50*pi/180]*np.sign[b1so]
   # Tt=min[abs[Tto],15]*np.sign[Tto]
   #Tt=Tto
   #####################################Tto###################################
   ###Sastry###

   
   # ###############
   # lm=-0.015
   # ym=0
   # hm=0.2943
   # ##############
   # ht=0.1154
   # lt=0.8715
   # #m=4.9
   # #############
   # Cqm=0.004452
   # Dqm=0.6304
   # dRmdb1s=25.23
   # #############
   # Cqt=0.005066
   # Dqt=0.008488
   # dMmda1s=25.23
   # Tmo=u1
   #######################################################################
   #######Jacobian Matrix for Rotational Dynamics############################

   dfpidu2 = 1/ixx
   dfpidu3 = (np.sin(pio)*np.tan(thetao))/iyy
   dfpidu4 = (np.cos(pio)*np.tan(thetao))/izz
   ##########
   dfthetadu2 = 0
   dfthetadu3 = np.cos(pio)/iyy
   dfthetadu4 = - (np.sin(pio)/izz)
   ##########
   dfpsidu2 = 0
   dfpsidu3 = np.sin(pio)/iyy/np.cos(thetao)
   dfpsidu4 = np.cos(pio)/np.cos(thetao)/izz

   ##########################################################################################

   w1=dpio-np.sin(thetao)*dpsio
   w2=np.cos(pio)*dthetao+(np.sin(pio)*np.cos(thetao))*dpsio
   w3=-np.sin(pio)*dthetao+(np.cos(pio)*np.cos(thetao))*dpsio
   #############################################################################################
   ###Disturbance for theroational dynamics####################################################
   dis_R_pi=np.cos(t)
   dis_R_theta=np.cos(t)
   dis_R_psi=np.cos(t)
   ####### Dynamic inversion for Rotational Dynamics ##################################
   ############################################################################################
   f1d=(np.cos(pio)*np.tan(thetao)*w2-np.sin(pio)*np.tan(thetao)*w3)*dpio \
      +(((1/np.cos(thetao))**2*np.sin(pio))*w2+(np.cos(pio)*(1/np.cos(thetao))**2)*w3)*dthetao \
      -((izz-iyy)/ixx)*w2*w3 \
      +(np.sin(pio)*np.tan(thetao))*((izz-ixx)/iyy)*w1*w3 \
      -(np.cos(pio)*np.tan(thetao))*((iyy-ixx)/izz)*w1*w2 \
      +(1/ixx)*u2 \
      +((np.sin(pio)*np.tan(thetao))/iyy)*u3 \
      +((np.cos(pio)*np.tan(thetao))/izz)*u4
   #########################################################
   f2d=-(np.sin(pio)*w2+np.cos(pio)*w3)*dpio \
      +np.cos(pio)*((izz-ixx)/iyy)*w1*w3 \
      +np.sin(pio)*((iyy-ixx)/izz)*w1*w2 \
      +(np.cos(pio)/iyy)*u3 \
      -(np.sin(pio)/izz)*u4
   ##########################################################
   f3d=(np.cos(pio)*1/np.cos(thetao)*w2-1/np.cos(thetao)*np.sin(pio)*w3)*dpio \
      +(1/np.cos(thetao)*np.sin(pio)*np.tan(thetao)*w2+np.cos(pio)*1/np.cos(thetao)*np.tan(thetao)*w3)*dthetao \
      +(1/np.cos(thetao)*np.sin(pio))*((izz-ixx)/iyy)*w1*w3 \
      -(np.cos(pio)*1/np.cos(thetao))*((iyy-ixx)/izz)*w1*w2 \
      +(1/np.cos (thetao)*np.sin(pio)/iyy)*u3 \
      +(np.cos(pio)*1/np.cos(thetao)/izz)*u4
   #################################################
   #### Design Extended High Gain Observer ######
   ############################################
   hpios=min(abs(x[24]),np.pi/2)*np.sign(x[24])
   hdpios=min(abs(x[25]),600)*np.sign(x[25])
   dis_hpios=min(abs(x[26]),2)*np.sign(x[26])
   # dis_hpios = 0
   hthetaos=min(abs(x[27]),np.pi/2)*np.sign(x[27])
   hdthetaos=min(abs(x[28]),80)*np.sign(x[28])
   dis_hthetaos=min(abs(x[29]),2)*np.sign(x[29])
   # dis_hthetaos = 0
   hpsis=min(abs(x[30]),np.pi/2)*np.sign(x[30])
   hdpsios=min(abs(x[31]),15)*np.sign(x[31])
   dis_hpsios=min(abs(x[32]),2)*np.sign(x[32])
   # dis_hpsios = 0
   h_dxs=min(abs(x[34]),2)*np.sign(x[34])
   h_dys=min(abs(x[37]),2)*np.sign(x[37])
   h_dzs=min(abs(x[40]),2)*np.sign(x[40])

   hpio=x[24]
   hdpio=x[25]

   hthetao=x[27]
   hdthetao=x[28]

   hpsi=x[30]
   hdpsio=x[31]
   ###########################################################################
   hw1s=hdpio-np.sin(hthetao)*hdpsios
   hw2s=np.cos(hpio)*hdthetaos+(np.sin(hpio)*np.cos(hthetao))*hdpsios
   hw3s=-np.sin(hpio)*hdthetaos+(np.cos(hpio)*np.cos(hthetao))*hdpsios

   ######################################################################################
   #############Rotaional dynamics for dynamic inversion#################################
   ######################################################################################
   # f1hs=[np.cos[hpios)*np.tan[hthetaos)*hw2s-np.sin[hpios)*np.tan[hthetaos)*hw3s)*hdpios \
   #    +[[[1/np.cos[hthetaos))**2*np.sin[hpios))*hw2s+[np.cos[hpios)*[1/np.cos[hthetaos))**2)*hw3s)*hdthetaos \
   #    -[[izz-iyy)/ixx)*hw2s*hw3s \
   #    +[np.sin[hpios)*np.tan[hthetaos))*[[izz-ixx)/iyy)*hw1s*hw3s \
   #    -[np.cos[hpios)*np.tan[hthetaos))*[[iyy-ixx)/izz)*hw1s*hw2s \
   #    +[1/ixx)*u2 \
   #    +[[np.sin[hpios)*np.tan[hthetaos))/iyy)*u3 \
   #    +[[np.cos[hpios)*np.tan[hthetaos))/izz)*u4
   # #########################################################
   # f2hs=-[np.sin[hpios)*hw2s+np.cos[hpios)*hw3s)*hdpios \
   #    +np.cos[hpios)*[[izz-ixx)/iyy)*hw1s*hw3s \
   #    +np.sin[hpios)*[[iyy-ixx)/izz)*hw1s*hw2s \
   #    +[np.cos[hpios)/iyy)*u3 \
   #    -[np.sin[hpios)/izz)*u4
   # ##########################################################
   # f3hs=[np.cos[hpios)*1/np.cos[hthetaos)*hw2s-1/np.cos[hthetaos)*np.sin[hpios)*hw3s)*hdpios \
   #    +[1/np.cos[hthetaos)*np.sin[hpios)*np.tan[hthetaos)*hw2s+np.cos[hpios)*1/np.cos[hthetaos)*np.tan[hthetaos)*hw3s)*hdthetaos \
   #    +[1/np.cos[hthetaos)*np.sin[hpios))*[[izz-ixx)/iyy)*hw1s*hw3s \
   #    -[np.cos[hpios)*1/np.cos[hthetaos))*[[iyy-ixx)/izz)*hw1s*hw2s \
   #    +[1/np.cos[hthetaos)*np.sin[hpios)/iyy)*u3 \
   #    +[np.cos[hpios)*1/np.cos[hthetaos)/izz)*u4
   # ##########################################################
   f1hs=(np.cos(pio)*np.tan(thetao)*hw2s-np.sin(pio)*np.tan(thetao)*hw3s)*hdpios \
      +(((1/np.cos(thetao))**2*np.sin(pio))*hw2s+(np.cos(pio)*(1/np.cos(thetao))**2)*hw3s)*hdthetaos \
      -((izz-iyy)/ixx)*hw2s*hw3s \
      +(np.sin(pio)*np.tan(thetao))*((izz-ixx)/iyy)*hw1s*hw3s \
      -(np.cos(pio)*np.tan(thetao))*((iyy-ixx)/izz)*hw1s*hw2s \
      +(1/ixx)*u2 \
      +((np.sin(pio)*np.tan(thetao))/iyy)*u3 \
      +((np.cos(pio)*np.tan(thetao))/izz)*u4
   #########################################################
   f2hs=-(np.sin(pio)*hw2s+np.cos(pio)*hw3s)*hdpios \
      +np.cos(pio)*((izz-ixx)/iyy)*hw1s*hw3s \
      +np.sin(pio)*((iyy-ixx)/izz)*hw1s*hw2s \
      +(np.cos(pio)/iyy)*u3 \
      -(np.sin(pio)/izz)*u4
   ##########################################################
   f3hs=(np.cos(pio)*1/np.cos(thetao)*hw2s-1/np.cos(thetao)*np.sin(pio)*hw3s)*hdpios \
      +(1/np.cos(thetao)*np.sin(pio)*np.tan(thetao)*hw2s+np.cos(pio)*1/np.cos(thetao)*np.tan(thetao)*hw3s)*hdthetaos \
      +(1/np.cos(thetao)*np.sin(pio))*((izz-ixx)/iyy)*hw1s*hw3s \
      -(np.cos(pio)*1/np.cos(thetao))*((iyy-ixx)/izz)*hw1s*hw2s \
      +(1/np.cos(thetao)*np.sin(pio)/iyy)*u3 \
      +(np.cos(pio)*1/np.cos(thetao)/izz)*u4
   ##############################################################################
   #pi_r=0.1
   #theta_r=0.1
   #psi_r=0
   # pi_r=u2
   # theta_r=u3
   # psi_r=AngPs

   # f1ErrorC=f1d-(-kpr*(pio-pi_v)-kvr*dpio)
   # f2ErrorC=f2d-(-kpr*(thetao-theta_v)-kvr*dthetao)
   # f3ErrorC=f3d-(-kpr*(psio-psi_r)-kvr*dpsio)

   # f1ErrorC=f1hs+dis_hpios-(-kpr*(pio-pi_v)-kvr*hdpios)
   # f2ErrorC=f2hs+dis_hthetaos-(-kpr*(thetao-theta_v)-kvr*hdthetaos)
   # f3ErrorC=f3hs+dis_hpsios-(-kpr*(psio-psi_r)-kvr*hdpsios)

   pi_v_test = 0
   theta_v_test = 0
   psi_r_test = 0

   # f1ErrorC=f1d+dis_R_pi-(-kpr*(pio-pi_v)-kvr*dpio)
   # f2ErrorC=f2d+dis_R_theta-(-kpr*(thetao-theta_v)-kvr*dthetao)
   # f3ErrorC=f3d+dis_R_psi-(-kpr*(psio-psi_r)-kvr*dpsio)
   f1ErrorC=f1hs+dis_R_pi - (-kpr*(pio-pi_v)-kvr*dpio)
   f2ErrorC=f2hs+dis_R_theta - (-kpr*(thetao-theta_v)-kvr*dthetao)
   f3ErrorC=f3hs+dis_R_psi - (-kpr*(psio-psi_r)-kvr*dpsio)

   ################################Translational dynamics#####################
   ###########################################################################
   f1x=(1/m)*(-(np.cos(pi_v)*np.sin(theta_v)*np.cos(psi_r)+np.sin(pi_v)*np.sin(psi_r))*u1)
   f2y=(1/m)*(-(np.cos(pi_v)*np.sin(theta_v)*np.sin(psi_r)-np.sin(pi_v)*np.cos(psi_r))*u1)
   f3z=(1/m)*(-(np.cos(pi_v)*np.cos(theta_v))*u1)+g

   ########
   f1xn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.cos(psi_r)+np.sin(pio)*np.sin(psi_r))*u1)
   f2yn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.sin(psi_r)-np.sin(pio)*np.cos(psi_r))*u1)
   f3zn=(1/m)*(-(np.cos(pio)*np.cos(thetao))*u1)+g
   ########
   # f1xn=(1/m)*(-np.cos(thetao)*np.cos(AngPs)*u1*np.sin(a1s)\
   #          +(np.sin(pio)*np.sin(thetao)*np.cos(AngPs)-np.cos(pio)*np.sin(AngPs))*(u1*np.sin(b1s)-Tt)\
   #          -(np.cos(pio)*np.sin(thetao)*np.cos(AngPs)+np.sin(pio)*np.sin(AngPs))*u1*np.cos(a1s)*np.cos(b1s))
   # f2yn=(1/m)*(-np.cos(thetao)*np.sin(AngPs)*u1*np.sin(a1s)\
   #          +(np.sin(pio)*np.sin(thetao)*np.sin(AngPs)+np.cos(pio)*np.cos(AngPs))*(u1*np.sin(b1s)-Tt)\
   #          -(np.cos(pio)*np.sin(thetao)*np.sin(AngPs)-np.sin(pio)*np.cos(AngPs))*u1*np.cos(a1s)*np.cos(b1s))
   ############################################################################
   ###########################################################################



   #############################################
   ####Extended High-Gain Observers dynamics
   ###########################################################################
   hf1xn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.cos(psi_r)+np.sin(pio)*np.sin(psi_r))*u1)
   hf2yn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.sin(psi_r)-np.sin(pio)*np.cos(psi_r))*u1)
   hf3zn=(1/m)*(-(np.cos(pio)*np.cos(thetao))*u1)+g
   # hf1xn=(1/m)*(-np.cos(thetao)*np.cos(AngPs)*u1*np.sin(a1s)\
   #          +(np.sin(pio)*np.sin(thetao)*np.cos(AngPs)-np.cos(pio)*np.sin(AngPs))*(u1*np.sin(b1s)-Tt)\
   #          -(np.cos(pio)*np.sin(thetao)*np.cos(AngPs)+np.sin(thetao)*np.sin(AngPs))*u1*np.cos(a1s)*np.cos(b1s))
   # hf2yn=(1/m)*(-np.cos(thetao)*np.sin(AngPs)*u1*np.sin(a1s)\
   #          +(np.sin(pio)*np.sin(thetao)*np.sin(AngPs)+np.cos(pio)*np.cos(AngPs))*(u1*np.sin(b1s)-Tt)\
   #          -(np.cos(pio)*np.sin(thetao)*np.sin(AngPs)-np.sin(pio)*np.cos(AngPs))*u1*np.cos(a1s)*np.cos(b1s))
   # hf3zn=(1/m)*(np.sin(thetao)*u1*np.sin(a1s)\
   #           +np.sin(pio)*np.cos(thetao)*(u1*np.sin(b1s)-Tt)\
   #           -(np.cos(pio)*np.cos(thetao))*u1*np.cos(a1s)*np.cos(b1s))+g

   #hf1xn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.cos(AngPs)+np.sin(pio)*np.sin(AngPs))*u1)
   #hf2yn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.sin(AngPs)-np.sin(pio)*np.cos(AngPs))*u1)
   # hf1xn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.cos(psio)+np.sin(pio)*np.sin(psio))*u1)
   # hf2yn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.sin(psio)-np.sin(pio)*np.cos(psio))*u1)
   # hf3zn=(1/m)*(-(np.cos(pio)*np.cos(thetao))*u1)+g
   ##########################################################################
   ###########################################################################


#     h_disxn=min(abs(x(36)),4)*np.sign(x(36))
#     h_disyn=min(abs(x(39)),4)*np.sign(x(39))
#     h_diszn=min(abs(x(42)),4)*np.sign(x(42))
   #############################################
   # f1hat=f1hn+x(22)
   # f2hat=f2hn+x(23)
   # f3hat=f3hn+x(24)
   # f1hat=min(abs(f1hn+x(22)),5)*np.sign(f1hn+x(22))
   # f2hat=min(abs(f2hn+x(23)),5)*np.sign(f2hn+x(23))
   # f3hat=min(abs(f3hn+x(24)),5)*np.sign(f3hn+x(24))


   ######Jacobian components for translational dynamics######################
   #####ideal cases##########################################################
   # f1u1n=(1/m)*(-np.cos(thetao)*np.cos(AngPs)*np.sin(a1s)\
   #      +(np.sin(pio)*np.sin(thetao)*np.cos(AngPs)-np.cos(pio)*np.sin(AngPs))*np.sin(b1s)\
   #     -(np.cos(pio)*np.sin(thetao)*np.cos(AngPs)+np.sin(thetao)*np.sin(AngPs))*np.cos(a1s)*np.cos(b1s))
   # 
   # f1u2n=(1/m)*((np.cos(pio)*np.sin(thetao)*np.cos(AngPs)+np.sin(pio)*np.sin(AngPs))*(u1*np.sin(b1s)-Tt)\
   #       +np.sin(pio)*np.sin(thetao)*np.cos(AngPs)*u1*np.cos(a1s)*np.cos(b1s))
   #   
   # f1u3n=(1/m)*(np.sin(thetao)*np.cos(AngPs)*u1*np.sin(a1s)\
   #      +np.sin(pio)*np.cos(thetao)*np.cos(AngPs)*(u1*np.sin(b1s)-Tt)\
   #      -(np.cos(pio)*np.cos(thetao)*np.cos(AngPs)+np.cos(thetao)*np.sin(AngPs))*u1*np.cos(a1s)*np.cos(b1s))
   # ########
   # f2u1n=(1/m)*(-np.cos(thetao)*np.sin(AngPs)*np.sin(a1s)\
   #      +(np.sin(pio)*np.sin(thetao)*np.sin(AngPs)+np.cos(pio)*np.cos(AngPs))*np.sin(b1s)\
   #      -(np.cos(pio)*np.sin(thetao)*np.sin(AngPs)-np.sin(pio)*np.cos(AngPs))*np.cos(a1s)*np.cos(b1s))
   #  
   # f2u2n=(1/m)*((np.cos(pio)*np.sin(thetao)*np.sin(AngPs)-np.sin(pio)*np.cos(AngPs))*(u1*np.sin(b1s)-Tt)\
   #     +(np.sin(pio)*np.sin(thetao)*np.sin(AngPs)+np.cos(pio)*np.cos(AngPs))*u1*np.cos(a1s)*np.cos(b1s))
   # 
   # f2u3n=(1/m)*(np.sin(thetao)*np.sin(AngPs)*u1*np.sin(a1s)\
   #      +np.sin(pio)*np.cos(thetao)*np.sin(AngPs)*(u1*np.sin(b1s)-Tt)\
   #      -np.cos(pio)*np.cos(thetao)*np.sin(AngPs)*u1*np.cos(a1s)*np.cos(b1s))
   # ########
   # f3u1n=(1/m)*(np.sin(thetao)*np.sin(a1s)\
   #     +np.sin(pio)*np.cos(thetao)*np.sin(b1s)\
   #     -np.cos(pio)*np.cos(thetao)*np.cos(a1s)*np.cos(b1s))
   # 
   # f3u2n=(1/m)*(np.cos(pio)*np.cos(thetao)*(u1*np.sin(b1s)-Tt)\
   #      +np.sin(pio)*np.cos(thetao)*u1*np.cos(a1s)*np.cos(b1s))
   #  
   # f3u3n=(1/m)*(np.cos(thetao)*u1*np.sin(a1s)\
   #     -np.sin(pio)*np.sin(thetao)*(u1*np.sin(b1s)-Tt)\
   #     +np.cos(pio)*np.sin(thetao)*u1*np.cos(a1s)*np.cos(b1s))
   ##########################################################################
   ##########################################################################
   # f1u1=(1/m)*(-np.cos(u3)*np.cos(AngPs)*np.sin(a1s)\
   #      +(np.sin(u2)*np.sin(u3)*np.cos(AngPs)-np.cos(u2)*np.sin(AngPs))*np.sin(b1s)\
   #     -(np.cos(u2)*np.sin(u3)*np.cos(AngPs)+np.sin(u3)*np.sin(AngPs))*np.cos(a1s)*np.cos(b1s))
   # f1u2=(1/m)*((np.cos(u2)*np.sin(u3)*np.cos(AngPs)+np.sin(u2)*np.sin(AngPs))*(u1*np.sin(b1s)-Tt)\
   #       +np.sin(u2)*np.sin(u3)*np.cos(AngPs)*u1*np.cos(a1s)*np.cos(b1s))
   # f1u3=(1/m)*(np.sin(u3)*np.cos(AngPs)*u1*np.sin(a1s)\
   #      +np.sin(u2)*np.cos(u3)*np.cos(AngPs)*(u1*np.sin(b1s)-Tt)\
   #      -(np.cos(u2)*np.cos(u3)*np.cos(AngPs)+np.cos(u3)*np.sin(AngPs))*u1*np.cos(a1s)*np.cos(b1s))

   # f1u1=(1/m)*(-(np.cos(u2)*np.sin(u3)*np.cos(AngPs)+np.sin(u3)*np.sin(AngPs)))
   # 
   # 
   # f1u2=(1/m)*(np.sin(u2)*np.sin(u3)*np.cos(AngPs)-np.cos(u2)*np.sin(AngPs))*u1
   # 
   #   
   # f1u3=-(1/m)*(np.cos(u2)*np.cos(u3)*np.cos(AngPs))*u1
   #########################################################################
   #########################################################################
   # f2u1=(1/m)*(-np.cos(u3)*np.sin(AngPs)*np.sin(a1s)\
   #      +(np.sin(u2)*np.sin(u3)*np.sin(AngPs)+np.cos(u2)*np.cos(AngPs))*np.sin(b1s)\
   #      -(np.cos(u2)*np.sin(u3)*np.sin(AngPs)-np.sin(u2)*np.cos(AngPs))*np.cos(a1s)*np.cos(b1s))
   # f2u2=(1/m)*((np.cos(u2)*np.sin(u3)*np.sin(AngPs)-np.sin(u2)*np.cos(AngPs))*(u1*np.sin(b1s)-Tt)\
   #     +(np.sin(u2)*np.sin(u3)*np.sin(AngPs)+np.cos(u2)*np.cos(AngPs))*u1*np.cos(a1s)*np.cos(b1s))
   # f2u3=(1/m)*(np.sin(u3)*np.sin(AngPs)*u1*np.sin(a1s)\
   #      +np.sin(u2)*np.cos(u3)*np.sin(AngPs)*(u1*np.sin(b1s)-Tt)\
   #      -np.cos(u2)*np.cos(u3)*np.sin(AngPs)*u1*np.cos(a1s)*np.cos(b1s))
   #  
   # f2u1=-(1/m)*(np.cos(u2)*np.sin(u3)*np.sin(AngPs)-np.sin(u2)*np.cos(AngPs))
   # 
   # f2u2=(1/m)*(np.sin(u2)*np.sin(u3)*np.sin(AngPs)+np.cos(u2)*np.cos(AngPs))*u1
   # 
   # f2u3=-(1/m)*(np.cos(u2)*np.cos(u3)*np.sin(AngPs))*u1
   ########
   # f3u1=(1/m)*(np.sin(u3)*np.sin(a1s)\
   #     +np.sin(u2)*np.cos(u3)*np.sin(b1s)\
   #     -np.cos(u2)*np.cos(u3)*np.cos(a1s)*np.cos(b1s))
   # f3u2=(1/m)*(np.cos(u2)*np.cos(u3)*(u1*np.sin(b1s)-Tt)\
   #      +np.sin(u2)*np.cos(u3)*u1*np.cos(a1s)*np.cos(b1s))
   # f3u3=(1/m)*(np.cos(u3)*u1*np.sin(a1s)\
   #     -np.sin(u2)*np.sin(u3)*(u1*np.sin(b1s)-Tt)\
   #     +np.cos(u2)*np.sin(u3)*u1*np.cos(a1s)*np.cos(b1s))
   #  
   # f3u1=(1/m)*(-np.cos(u2)*np.cos(u3))
   # 
   # f3u2=(1/m)*(np.sin(u2)*np.cos(u3)*u1)

   # f3u3=(1/m)*(np.cos(u3)*u1*np.sin(a1s)\
   #      -np.sin(u2)*np.sin(u3)*(u1*np.sin(b1s)-Tt)\
   #      +np.cos(u2)*np.sin(u3)*u1*np.cos(a1s)*np.cos(b1s))
   # pio thetao
   # f3u3=(1/m)*(np.cos(u2)*np.sin(u3)*u1)
   #f3u3=(1/m)*(np.cos(pio)*np.sin(thetao)*u1)
   ############################################

   x1_temp = x[0]
   x2_temp = x[1]
   x3_temp = x[2]
   x4_temp = x[3]
   x5_temp = x[4]
   x6_temp = x[5]
   x7_temp = x[6]
   x8_temp = x[7]
   x9_temp = x[8]
   x10_temp = x[9]
   x11_temp = x[10]
   x12_temp = x[11]
   x13_temp = x[12]
   x14_temp = x[13]
   x15_temp = x[14]
   x16_temp = x[15]
   x17_temp = x[16]
   x18_temp = x[17]
   x19_temp = x[18]
   x20_temp = x[19]
   x21_temp = x[20]
   x22_temp = x[21]
   x23_temp = x[22]
   x24_temp = x[23]
   x25_temp = x[24]
   x26_temp = x[25]
   x27_temp = x[26]
   x28_temp = x[27]
   x29_temp = x[28]
   x30_temp = x[29]
   x31_temp = x[30]
   x32_temp = x[31]
   x33_temp = x[32]
   x34_temp = x[33]
   x35_temp = x[34]
   x36_temp = x[35]
   x37_temp = x[36]
   x38_temp = x[37]
   x39_temp = x[38]
   x40_temp = x[39]
   x41_temp = x[40]
   x42_temp = x[41]
   x43_temp = x[42]
   x44_temp = x[43]
   x45_temp = x[44]
   x46_temp = x[45]
   x47_temp = x[46]
   x48_temp = x[47]
   x49_temp = x[48]
   x50_temp = x[49]
   x51_temp = x[50]
   
   # for q in range(51):
   #    print(x[q])

   # Translational
   x[0] = x1_temp + dt*x4_temp
   x[1] = x2_temp + dt*x5_temp
   x[2] = x3_temp + dt*x6_temp
   x[3] = x4_temp + dt*(f1x+3*np.sin(t))
   x[4] = x5_temp + dt*(f2y+3*np.sin(t))
   x[5] = x6_temp + dt*(f3z+3*np.sin(t))

   # References
   x[9]  = x10_temp+dt*x13_temp
   x[10] = x11_temp+dt*x14_temp
   x[11] = x12_temp+dt*x15_temp
   x[12] = x13_temp + dt*(-kp*(x10_temp-5*np.sin(t)) - kv*(x13_temp-5*np.cos(t)) + r1)
   x[13] = x14_temp + dt*(-kp*(x11_temp-5*np.cos(t)) - kv*(x14_temp+5*np.sin(t)) + r2)
   x[14] = x15_temp + dt*(-kpz*(x12_temp-5*np.sin(t)) - kvz*(x15_temp-5*np.cos(t)) + r3)
   # x[14] = x15_temp + dt*(-kpz*(x12_temp-5.) - kvz*(x15_temp-0.) + r3)

   # Rotational
   x[15] = x16_temp + dt*x19_temp
   x[16] = x17_temp + dt*x20_temp
   x[17] = x18_temp + dt*x21_temp
   x[18] = x19_temp+dt*(f1d+dis_R_pi)
   x[19] = x20_temp+dt*(f2d+dis_R_theta)
   x[20] = x21_temp+dt*(f3d+dis_R_psi)


   # Dynamic Inversion
   fErrorMat = np.array([[f1ErrorC], [f2ErrorC], [f3ErrorC]])

   # print(fErrorMat)

   dfduMat = np.array([[dfpidu2, dfpidu3, dfpidu4], \
                  [dfthetadu2, dfthetadu3, dfthetadu4], \
                  [dfpsidu2, dfpsidu3, dfpsidu4] \
                  ])
   # print(np.shape(fErrorMat))

   duMat = -(1/elip_Rotaional)*multi_dot([np.transpose(dfduMat),np.transpose(PMat),fErrorMat])
   
   dPMat = -1*multi_dot([PMat,np.transpose(dfduMat),dfduMat,PMat])
   # print(duMat)
   # print('shape : ',duMat)

   # U2, U3, U4
   x[21] = x22_temp + dt*duMat[0,0]
   x[22] = x23_temp + dt*duMat[1,0]
   x[23] = x24_temp + dt*duMat[2,0]

   # Low pass filter
   u2o = x[21]
   u3o = x[22]
   u4o = x[23]

   u2f = (1-a)*x22_temp + a*u2o
   u3f = (1-a)*x23_temp + a*u3o
   u4f = (1-a)*x24_temp + a*u4o

   # Update values
   x[21] = u2f
   x[22] = u3f
   x[23] = u4f
   u2 = u2f
   u3 = u3f
   u4 = u4f

   
   # P Matrix
   x[42] = x43_temp+ dt*dPMat[0,0]
   x[43] = x44_temp+ dt*dPMat[0,1]
   x[44] = x45_temp+ dt*dPMat[0,2]
   x[45] = x46_temp+ dt*dPMat[1,0]
   x[46] = x47_temp+ dt*dPMat[1,1]
   x[47] = x48_temp+ dt*dPMat[1,2]
   x[48] = x49_temp+ dt*dPMat[2,0]
   x[49] = x50_temp+ dt*dPMat[2,1]
   x[50] = x51_temp+ dt*dPMat[2,2]


   # ###########################################################################
   hw1=hdpio-np.sin(hthetao)*hdpsio
   hw2=np.cos(hpio)*hdthetao+(np.sin(hpio)*np.cos(hthetao))*hdpsio
   hw3=-np.sin(hpio)*hdthetao+(np.cos(hpio)*np.cos(hthetao))*hdpsio
   #####################################################################################
   #####################################################################################
   ######################################
   f1h=(np.cos(hpio)*np.tan(hthetao)*hw2-np.sin(hpio)*np.tan(hthetao)*hw3)*hdpio \
      +(((1/np.cos(hthetao))**2*np.sin(hpio))*hw2+(np.cos(hpio)*(1/np.cos(hthetao))**2)*hw3)*hdthetao \
      -((izz-iyy)/ixx)*hw2*hw3 \
      +(np.sin(hpio)*np.tan(hthetao))*((izz-ixx)/iyy)*hw1*hw3 \
      -(np.cos(hpio)*np.tan(hthetao))*((iyy-ixx)/izz)*hw1*hw2 \
      +(1/ixx)*u2 \
      +((np.sin(hpio)*np.tan(hthetao))/iyy)*u3 \
      +((np.cos(hpio)*np.tan(hthetao))/izz)*u4
   #########################################################
   f2h=-(np.sin(hpio)*hw2+np.cos(hpio)*hw3)*hdpio \
      +np.cos(hpio)*((izz-ixx)/iyy)*hw1*hw3 \
      +np.sin(hpio)*((iyy-ixx)/izz)*hw1*hw2 \
      +(np.cos(hpio)/iyy)*u3 \
      -(np.sin(hpio)/izz)*u4
   ##########################################################
   f3h=(np.cos(hpio)*1/np.cos(hthetao)*hw2-1/np.cos(hthetao)*np.sin(hpio)*hw3)*hdpio \
      +(1/np.cos(hthetao)*np.sin(hpio)*np.tan(hthetao)*hw2+np.cos(hpio)*1/np.cos(hthetao)*np.tan(hthetao)*hw3)*hdthetao \
      +(1/np.cos(hthetao)*np.sin(hpio))*((izz-ixx)/iyy)*hw1*hw3 \
      -(np.cos(hpio)*1/np.cos(hthetao))*((iyy-ixx)/izz)*hw1*hw2 \
      +(1/np.cos(hthetao)*np.sin(hpio)/iyy)*u3 \
      +(np.cos(hpio)*1/np.cos(hthetao)/izz)*u4
   #######Dynamics of Extended High-Gain Observer for Rotational Dynamics################
   # dx(25)=x(26)+(alpha11/elipH)*(x(16)-x(25))
   # dx(26)=f1h+x(27)+(alpha12/elipH**2)*(x(16)-x(25))
   # dx(27)=(alpha13/elipH**3)*(x(16)-x(25))
   x[24]=x25_temp + dt*(x26_temp+(alpha11/elipHr)*(x16_temp-x25_temp))
   x[25]=x26_temp + dt*(f1h+x27_temp+(alpha12/elipHr**2)*(x16_temp-x25_temp))
   x[26]=x27_temp + dt*((alpha13/elipHr**3)*(x16_temp-x25_temp))
   #####Measurement:  x[17]:\theteta##################################################################
   ###Theta Extended_High_Gain_Observer#########
   # dx[28]=x[29]+[alpha21/elipH]*[x[17]-x[28]]
   # dx[29]=f2h+x[30]+[alpha22/elipH**2]*[x[17]-x[28]]
   # dx(30]=(alpha23/elipH**3]*(x[17]-x[28]]
   x[27]=x28_temp + dt*(x29_temp+(alpha21/elipHr)*(x17_temp-x28_temp))
   x[28]=x29_temp + dt*(f2h+x30_temp+(alpha22/elipHr**2)*(x17_temp-x28_temp))
   x[29]=x30_temp + dt*((alpha23/elipHr**3)*(x17_temp-x28_temp))
   #####Measurement: x[18]:\psi####################
   ###Psi Extended_High_Gain_Observer#########
   # dx[31]=x[32]+[alpha31/elipH]*[x[18]-x[31]]
   # dx[32]=f3h+x[33]+[alpha32/elipH**2]*[x[18]-x[31]]
   # dx[33]=[alpha33/elipH**3]*[x[18]-x[31]]
   x[30]=x31_temp + dt*(x32_temp+(alpha31/elipHr)*(x18_temp-x31_temp))
   x[31]=x32_temp + dt*(f3h+x33_temp+(alpha32/elipHr**2)*(x18_temp-x31_temp))
   x[32]=x33_temp + dt*(alpha33/elipHr**3)*(x18_temp-x31_temp)
   # ##################################################################
   # #########################################################################
   #######Design of Extended High-Gain Observers for Translational dynamics########
   ######################################################################
   x[33]=x34_temp+dt*(x35_temp+(alpha11p/elipHt)*(x1_temp-x34_temp))
   x[34]=x35_temp+dt*(hf1xn+x36_temp+(alpha21p/elipHt**2)*(x1_temp-x34_temp))
   x[35]=x36_temp+dt*((alpha31p/elipHt**3)*(x1_temp-x34_temp))
   ###### Y-dynamics#################################
   # dx[37]=x[38]+[alpha11p/elipH]*[x[2]-x[37]]
   # dx[38]=hf2yn+x[39]+[alpha21p/elipH**2]*[x[2]-x[37]]
   # dx[39]=[alpha31p/elipH**3]*[x[2]-x[37]]
   x[36]=x37_temp+dt*(x38_temp+(alpha11p/elipHt)*(x2_temp-x37_temp))
   x[37]=x38_temp+dt*(hf2yn+x39_temp+(alpha21p/elipHt**2)*(x2_temp-x37_temp))
   x[38]=x39_temp+dt*(alpha31p/elipHt**3)*(x2_temp-x37_temp)
   ######## Z-dynamics ###############################
   ######Extended High Gain Observer######
   # dx[40]=x[41]+[alpha11p/elipH]*[x[3]-x[40]]
   # dx[41]=hf3zn+x[42]+[alpha21p/elipH**2]*[x[3]-x[40]]
   # dx[42]=[alpha31p/elipH**3]*[x[3]-x[40]]
#     x[40]=x40_temp+dt*[x41_temp+[alpha11p/elipH]*[x3_temp-x40_temp]]
#     x[41]=x41_temp+dt*[hf3zn+x42_temp+[alpha21p/elipH**2]*[x3_temp-x40_temp]]
#     x[42]=x42_temp+dt*[[alpha31p/elipH**3]*[x3_temp-x40_temp]]
   x[39]=x40_temp+dt*(x41_temp+(beta11p/elipHt)*(x3_temp-x40_temp))
   x[40]=x41_temp+dt*(hf3zn+x42_temp+(beta21p/elipHt**2)*(x3_temp-x40_temp))
   x[41]=x42_temp+dt*((beta31p/elipHt**3)*(x3_temp-x40_temp))
   ########################################

   d1 = 0.1572
   d2 = 0.1572
   c = 0.1

   x[51]= f1hs
   x[52]= dis_R_pi
   x[53]= (-kpr*(pio-pi_v)-kvr*dpio)

   x[54]= f3ErrorC



##


x_1 = np.array(X[:,0])
y_1 = np.array(X[:,1])
z_1 = np.array(X[:,2])

x_2 = np.array(X[:,3])
y_2 = np.array(X[:,4])
z_2 = np.array(X[:,5])

m=1.157
g=9.81

pio=np.array(X[:,15])
thetao=np.array(X[:,16])
psio=np.array(X[:,17])
dpio=np.array(X[:,18])
dthetao=np.array(X[:,19])
dpsio=np.array(X[:,20])


AngPs = 0.01*np.sin(T*0.01)
psi_r = AngPs

r1= - 5*np.array(np.sin(T))
r2= - 5*np.array(np.cos(T))
r3= - 5*np.array(np.sin(T))
# r3 = 0.

sig_x = 3*np.array(np.sin(T))
sig_y = 3*np.array(np.sin(T))
sig_z = 3*np.array(np.sin(T))

f_x = -kp*(x_1-5*np.array(np.sin(T))) - kv*(x_2-5*np.array(np.cos(T))) + r1 - sig_x
f_y = -kp*(y_1-5*np.array(np.cos(T))) - kv*(y_2+5*np.array(np.sin(T))) + r2 - sig_y
f_z = -kp*(z_1-5*np.array(np.sin(T))) - kv*(z_2-5*np.array(np.cos(T))) + r3 - sig_z
# f_z = -kp*(z_1-5.) - kv*(z_2-0.) + r3 - sig_z




### DEFINE U1 [Feedback LinearizaTion]
u1 = - (m*(f_z - g)) / (np.cos(thetao)*np.cos(pio))

# ### DEFINE THETA_V
y_th = f_y*np.sin(psi_r) + f_x*np.cos(psi_r)
x_th = f_z-g
theta_v = np.arctan(y_th/x_th)

# ### DEFINE PI_V
y_pi = f_x*np.cos(theta_v)*np.sin(psi_r)-f_y*np.cos(theta_v)*np.cos(psi_r)
x_pi = f_z-g
pi_v = np.arctan(y_pi/x_pi)


hatx=np.array([min(abs(X[i,33]),10)*np.sign(X[i,33]) for i in range(Hz*timespan)])
haty=np.array([min(abs(X[i,36]),10)*np.sign(X[i,36]) for i in range(Hz*timespan)])
hatz=np.array([min(abs(X[i,39]),10)*np.sign(X[i,39]) for i in range(Hz*timespan)])

hatdx=np.array([min(abs(X[i,34]),10)*np.sign(X[i,34]) for i in range(Hz*timespan)])
hatdy=np.array([min(abs(X[i,37]),10)*np.sign(X[i,37]) for i in range(Hz*timespan)])
hatdz=np.array([min(abs(X[i,40]),10)*np.sign(X[i,40]) for i in range(Hz*timespan)])


##
plt.figure(1)
plt.subplot(311)
plt.plot(T,X[:,0],T,5*np.sin(T),T,hatx)
# plt.plot(T,hatx)
# title('Translational Dynamics')
# plt.xlabel('t (1/np.cos)') 
# plt.ylabel('x_1 and x_r')

plt.subplot(312)
plt.plot(T,X[:,1],T,5*np.cos(T),T,haty)
# plt.plot(T,haty)
# plt.xlabel('t (1/np.cos)') 
# plt.ylabel('y_1 and y_r')

plt.subplot(313)
plt.plot(T,X[:,2],T,5*np.sin(T),T,hatz)
# plt.plot(T,X[:,2],T,5.*np.ones(250*10),T,hatz)
# plt.plot(T,hatz)
# plt.xlabel('t (1/np.cos)')  
# plt.ylabel('z_1 and z_r')

plt.show()


plt.figure(2)
plt.subplot(311)
plt.plot(T,X[:,3],T,X[:,12],T,hatdx)
# plt.subplot(311)plt.plot(T,X[:,4],T,X[:,13)]
plt.xlabel('t (1/np.cos)')
plt.ylabel('x_2 and Ref: dx_r ')

plt.subplot(312)
plt.plot(T,X[:,4],T,X[:,13],T,hatdy)
# plt.subplot(312)plt.plot(T,X[:,5],T,X[:,14)]
plt.xlabel('t (1/np.cos)')
plt.ylabel('y_2 and Ref: dy_r ')

plt.subplot(313)
plt.plot(T,X[:,5],T,X[:,14],T,hatdz)
# plt.subplot(313)plt.plot(T,X[:,6],T,X[:,15)]
plt.xlabel('t (1/np.cos)')
plt.ylabel('z_2 and Ref: dz_r')

plt.show()


# ####Verification of the Extended High Gain Observer########
m=1.175
g=9.8
# #####nominal rotational dynamics################################
f1V=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.cos(psi_r)+np.sin(pio)*np.sin(psi_r))*u1)+3*np.sin(T)
f2V=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.sin(psi_r)-np.sin(pio)*np.cos(psi_r))*u1)+3*np.sin(T)
f3V=(1/m)*(-(np.cos(pio)*np.cos(thetao))*u1)+g+3*np.sin(T)
# ###########################################################
Oh_disxn=np.array([min(abs(X[i,35]),5)*np.sign(X[i,35]) for i in range(Hz*timespan)])
Oh_disyn=np.array([min(abs(X[i,38]),5)*np.sign(X[i,38]) for i in range(Hz*timespan)])
Oh_diszn=np.array([min(abs(X[i,41]),5)*np.sign(X[i,41]) for i in range(Hz*timespan)])
##########################################################
hatPhi=np.array([min(abs(X[i,24]),np.pi/2.)*np.sign(X[i,24]) for i in range(Hz*timespan)])
hatTheta=np.array([min(abs(X[i,27]),np.pi/2.)*np.sign(X[i,27]) for i in range(Hz*timespan)])
hatPsi=np.array([min(abs(X[i,30]),np.pi/2.)*np.sign(X[i,30]) for i in range(Hz*timespan)])

hatdPhi=np.array([min(abs(X[i,25]),600.)*np.sign(X[i,25]) for i in range(Hz*timespan)])
hatdTheta=np.array([min(abs(X[i,28]),80.)*np.sign(X[i,28]) for i in range(Hz*timespan)])
hatdPsi=np.array([min(abs(X[i,31]),15.)*np.sign(X[i,31]) for i in range(Hz*timespan)])
################# #########################################
# f1Vc=(1/m)*(-(np.cos(Rphi)*np.sin(Rtheta)*np.cos(AngPsV)+np.sin(Rphi)*np.sin(AngPsV))*u1V)+Oh_disxn
# f2Vc=(1/m)*(-(np.cos(Rphi)*np.sin(Rtheta)*np.sin(AngPsV)-np.sin(Rphi)*np.cos(AngPsV))*u1V)+Oh_disyn
# f3Vc=(1/m)*(-(np.cos(Rphi)*np.cos(Rtheta))*u1V)+g+Oh_diszn

f1Vc=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.cos(psi_r)+np.sin(pio)*np.sin(psi_r))*u1)+Oh_disxn
f2Vc=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.sin(psi_r)-np.sin(pio)*np.cos(psi_r))*u1)+Oh_disyn
f3Vc=(1/m)*(-(np.cos(pio)*np.cos(thetao))*u1)+g+Oh_diszn

plt.figure(5)
plt.subplot(311) 
plt.plot(T,X[:,15],'k-',T,hatPhi,'k--')
#title('\phi_1, \phi_d')
plt.axis([-0.3, 10, -2,  2])
plt.xlabel('t (1/np.cos)')
plt.ylabel('\phi_1, Est: \phi_1')
#y label('state variable x and reference xr')

plt.subplot(312) 
plt.plot(T,X[:,16],'k-',T,hatTheta,'k--')
plt.axis([-0.3, 10, -2,  2])
plt.xlabel('t (1/np.cos)')
plt.ylabel('\theta_1, Est: \theta_1')

plt.subplot(313) 
plt.plot(T,X[:,17],'k-',T,hatPsi,'k--')
plt.axis([-0.3, 10, -2,  2])
plt.xlabel('t (1/np.cos)')
plt.ylabel('\psi_1, Est: \psi_1')
# # 
# 
plt.show()


plt.figure(6)
plt.subplot(311) 
plt.plot(T,X[:,18],'k-',T,hatdPhi,'k--')
# plt.subplot(311) plt.plot(T,hatdPhi,'k--')
# plt.axis((-0.3 10 -1  1))
plt.xlabel('t (1/np.cos)')
plt.ylabel('\phi_2 and Est: \phi_2')
#plt.subplot(311) plt.plot(T,X[:,19)]
#title('\phi, Ets:\phi')
#plt.ylabel('state variable x and reference xr')

plt.subplot(312) 
plt.plot(T,X[:,19],'k-',T,hatdTheta,'k--')
# plt.axis((-0.3 10 -6  6))
plt.xlabel('t (1/np.cos)')
plt.ylabel('\theta_2 and Est: \theta_2')

plt.subplot(313) 
plt.plot(T,X[:,20],'k-',T,hatdPsi,'k--')
# plt.axis((-0.3 10 -1  1))
#plt.subplot(313) plt.plot(T,X[:,21)]
plt.xlabel('t (1/np.cos)')
plt.ylabel('\psi_2 and Est: \psi_2')
# 
# # Oh_disxn=min(abs(X[:,36)],5)*np.sign(X[:,36))
# # Oh_disyn=min(abs(X[:,39)],5)*np.sign(X[:,39))
# # Oh_diszn=min(abs(X[:,42)],5)*np.sign(X[:,42))

plt.show()



plt.figure(7)
plt.subplot(311)
plt.plot(T,f1Vc,'k--',T,f1V,'k-')
# plt.axis((-0.3 10 -15  15))
plt.xlim([-0.3, 10])
plt.xlabel('t (1/np.cos)')
plt.ylabel('Dis: x and Est: x')


plt.subplot(312) 
plt.plot(T,f2Vc,'k--',T,f2V,'k-')
# plt.axis((-0.3 10 -15  15))
plt.xlim([-0.3, 10])
plt.xlabel('t (1/np.cos)')
plt.ylabel('Dis: y and Est: y')

plt.subplot(313) 
plt.plot(T,f3Vc,'k--',T,f3V,'k-')
# plt.axis((-0.3 10 -15  15))
plt.xlim([-0.3, 10])
plt.xlabel('t (1/np.cos)')
plt.ylabel('Dis: z and Est: z')

plt.show()

##
# ######x and x_r#####graph####
plt.figure(8)
plt.subplot(311)
plt.plot(T,X[:,0],'k-',T,5*np.sin(T),'k--')
#title('Translational Dynamics')
# plt.axis((-0.3 10 -2  2))
plt.xlabel('t (1/np.cos)') 
plt.ylabel('x_1 and x_r')

plt.subplot(312)
plt.plot(T,X[:,1],'k-',T,5*np.cos(T),'k--')
# plt.axis((-0.3 10 -2  2))
plt.xlabel('t (1/np.cos)') 
plt.ylabel('y_1 and y_r')

plt.subplot(313)
plt.plot(T,X[:,2],'k-',T,5*np.sin(T),'k--')
# plt.plot(T,X[:,2],'k-',T,5.*np.ones(250*10),'k--')
# plt.axis((-0.3 10 -2  2))
plt.xlabel('t (1/np.cos)')  
plt.ylabel('z_1 and z_r')

plt.show()

##
# ###########phi theta psi phi_r theta_r psi_r###############
plt.figure(9)
plt.subplot(311) 
plt.plot(T,X[:,15],'k-',T,pi_v,'k--')
plt.axis([-0.3, 10, -2,  2])
#title('\phi_1, \phi_d')
plt.xlabel('t (1/np.cos)')
plt.ylabel('\phi_1, \phi_d')
#y label('state variable x and reference xr')

plt.subplot(312) 
plt.plot(T,X[:,16],'k-',T,theta_v,'k--')
plt.axis([-0.3, 10, -2,  2])
plt.xlabel('t (1/np.cos)')
plt.ylabel('\theta_1, \theta_d')

plt.subplot(313) 
plt.plot(T,X[:,17],'k-',T,psi_r,'k--')
plt.axis([-0.3, 10, -2,  2])
plt.xlabel('t (1/np.cos)')
plt.ylabel('\psi_1, \psi_r')

plt.show()

##
plt.figure(10)
plt.subplot(311)
plt.plot(T,X[:,0],'k-',T,hatx,'k--')
# plt.axis((-0.3 10 -2  2))
plt.xlabel('t (1/np.cos)') 
plt.ylabel('x_1 and Est: x_1')

plt.subplot(312)
plt.plot(T,X[:,1],'k-',T,haty,'k--')
# plt.axis((-0.3 10 -2  2))
plt.xlabel('t (1/np.cos)') 
plt.ylabel('y_1 and Est: y_1')

plt.subplot(313)
plt.plot(T,X[:,2],'k-',T,hatz,'k--')
# plt.axis((-0.3 10 -2  2))
plt.xlabel('t (1/np.cos)')  
plt.ylabel('z_1 and Est: z_1')

plt.show()

##
plt.figure(11)
plt.subplot(311)
plt.plot(T,X[:,3],'k-',T,hatdx,'k--')
# plt.axis((-0.3 10 -10  10))
plt.xlabel('t (1/np.cos)')
plt.ylabel('dx_2 and Est: dx_2 ')

plt.subplot(312)
plt.plot(T,X[:,4],'k-',T,hatdy,'k--')
# plt.axis((-0.3 10 -10  10))
plt.xlabel('t (1/np.cos)')
plt.ylabel('dy_2 and Est: dy_2')

plt.subplot(313)
plt.plot(T,X[:,5],'k-',T,hatdz,'k--')
# plt.axis((-0.3 10 -10  10))
plt.xlabel('t (1/np.cos)')
plt.ylabel('dz_2 and Est: dz_2')

plt.show()

# 
# ######################################################################
# ###################disturbances for rotional dynamics######
Rdis_hpios=np.array([min(abs(X[i,26]),2)*np.sign(X[i,26]) for i in range(Hz*timespan)])
Rdis_hthetaos=np.array([min(abs(X[i,29]),2)*np.sign(X[i,29]) for i in range(Hz*timespan)])
Rdis_hpisos=np.array([min(abs(X[i,32]),2)*np.sign(X[i,32]) for i in range(Hz*timespan)])

# ####################################################################
# ###############disturbance model for rotational dynamics###################
DISphi=np.cos(T)
DIStheta=np.cos(T)
DISpis=np.cos(T)


####################################################################
plt.figure(12)
plt.subplot(311)
plt.plot(T,Rdis_hpios,'k--',T,DISphi,'k-'),
# plt.subplot(311)plt.plot(T,DISphi,'k-'),
plt.axis([-0.3, 10, -3,  3])
plt.xlabel('t (1/np.cos)')
plt.ylabel('Dist: \phi  and Est: \phi')
 
plt.subplot(312)
plt.plot(T,Rdis_hthetaos,'k--',T,DIStheta,'k-'),
# plt.subplot(312)plt.plot(T,DIStheta,'k-'),
plt.axis([-0.3, 10, -3,  3])
plt.xlabel('t (1/np.cos)')
plt.ylabel('Dist: \theta and Est: \theta')

plt.subplot(313)
plt.plot(T,Rdis_hpisos,'k--',T,DISpis,'k-'),
# plt.subplot(313)plt.plot(T,DISpis,'k-'),
plt.axis([-0.3, 10, -3,  3])
plt.xlabel('t (1/np.cos)')
plt.ylabel('Dist: \psi and Est: \psi')
# 
plt.show()

# plt.figure(13)
# plt.subplot(311)
# plt.plot(T,X[:,51])

# plt.subplot(312)
# plt.plot(T,X[:,52])

# plt.subplot(313)
# plt.plot(T,X[:,53])

# # plt.subplot(414)
# # plt.plot(T,X[:,54])
# # 

# plt.show()
