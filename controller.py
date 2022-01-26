import numpy as np
from numpy.linalg import multi_dot


### Success List
## 1
# elip_Rotaional=0.000003
# PInit = 0.0000001

## 2
# elip_Rotaional=0.000005
# PInit = 0.0000004
# dtf = 0.55

## 3
# elip_Rotaional=0.00001
# PInit = 0.000001
# dtf = 0.55

## 4
# elip_Rotaional=0.00004
# PInit = 0.00001
# dtf = 0.55

## 5
# elip_Rotaional=0.0005
# PInit = 0.000
# dtf = 0.55

## 6
# elip_Rotaional=0.0006
# PInit = 0.00085
# dtf = 0.51 

## 7
# elip_Rotaional=0.0006
# PInit = 0.00088
# dtf = 0.51 

elip_Rotaional=0.3
PInit = 10.

Hz = 250
dt = 1./Hz
 
# dtf = 0.46

a = 0.1

t = 0.

m=1.
# ixx = 0.02024*0.46
# iyy = 0.01359*0.46
# izz = 0.03626*0.46
ixx = 0.00168*9.81*10./12.
iyy = 0.00168*9.81*10./12.
izz = 0.0031*9.81*10./12.
g=9.8


# Rotor Parameters
C_T = 0.109919
rho = 1.225
D = 0.2286

# 1 x y z positions 3
# 4 vx vy vz velocities6
# 7 inputs phi theta psi 9
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


# Parameters for EHGOs
# Rotational EHGO
elipHr=0.015
alpha11=6
alpha12=6
alpha13=2
alpha21=6
alpha22=6
alpha23=2
alpha31=6
alpha32=6
alpha33=2
# elipH=0.05
# alpha11=1.5
# alpha12=0.74
# alpha13=0.12
# alpha21=alpha11
# alpha22=alpha12
# alpha23=alpha13
# alpha31=alpha11
# alpha32=alpha12
# alpha33=alpha13
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
# elipr=0.01
# kpr=50/elipr**2
# kvr=10/elipr



# elipr=0.5
# kpr=50/elipr**2
# kvr=10/elipr
kpr = 50./0.9/0.9
kvr = 10./0.9


def RLS_controller(t,flight_mode,x,x_p):
   ########################### Current States #############################
   # Kinematics
   x_1 = x[0]
   y_1 = x[1]
   z_1 = x[2]

   x_2 = x[3]
   y_2 = x[4]
   z_2 = x[5]

   
   
   pio = x[15]
   thetao = x[16]
   psio = x[17]
   dpio = x[18]
   dthetao = x[19]
   dpsio = x[20]


   # Disturbances
   sig_x = x[35]
   sig_y = x[36]
   sig_z = x[37]

   # Control inputs
   u2 = x[21]
   u3 = x[22]
   u4 = x[23]

   # RLS
   p11=x[42]
   p12=x[43]
   p13=x[44]
   p21=x[45]
   p22=x[46]
   p23=x[47]
   p31=x[48]
   p32=x[49]
   p33=x[50]

   # EHGOs
   hpio=x[24]
   hdpio=x[25]

   hthetao=x[27]
   hdthetao=x[28]

   hpsi=x[30]
   hdpsio=x[31]

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


   ########################### Translational References ############################
   if flight_mode == 'Auto':
      # len_row, len_clo = np.shape(xtraj)
      # flag = math.round(k+10)/10.

      # if g < len_row:
      #    x_r = xtraj(g)/100 + x_offset
      #    y_r = ytraj(g)/100 + y_offset
      #    z_r = z_offset
      # else:
      # x_r = 0.0333*Tx_r + x_r_ # Gives max speed of 2m/s
      # y_r = 0.0333*Ty_r + y_r_ # Gives max speed of 2m/s
      # z_r = -0.0003333333*(Tz_r-15) + z_r_
      x_r = + 5*np.sin(t)
      y_r = + 5*np.cos(t)
      z_r = + 5*np.sin(t)

      x2_r = + 5*np.cos(t)
      y2_r = - 5*np.sin(t)
      z2_r = + 5*np.cos(t)

      x3_r = - 5*np.sin(t)
      y3_r = - 5*np.cos(t)
      z3_r = - 5*np.sin(t)

   elif flight_mode == 'Manual':
      x_r = x_1
      y_r = y_1
      z_r = z_1


   #################################################################################

   ########################### Translational Control ############################

   AngPs = 0.01*np.sin(t*0.01)
   psi_r = AngPs

   f_x = -kp*(x_1-x_r) - kv*(x_2-x2_r) + x3_r - sig_x
   f_y = -kp*(y_1-y_r) - kv*(y_2-y2_r) + y3_r - sig_y
   f_z = -kpz*(z_1-z_r) - kvz*(z_2-z2_r) + z3_r - sig_z

   ### DEFINE THETA_V
   y_th = f_y*np.sin(psi_r) + f_x*np.cos(psi_r)
   x_th = f_z-g
   theta_v = np.arctan(y_th/x_th)
   # theta_v = min(abs(theta_v),1.25)*np.sign(theta_v)

   ### DEFINE PI_V
   y_pi = f_x*np.cos(theta_v)*np.sin(psi_r)-f_y*np.cos(theta_v)*np.cos(psi_r)
   x_pi = f_z-g
   pi_v = np.arctan(y_pi/x_pi)
   # pi_v = min(abs(pi_v),1.25)*np.sign(pi_v)

   u1 = - (m*(f_z - g)) / (np.cos(theta_v)*np.cos(pi_v))

   ###########################################################################

   ########################### Rotational Control ############################
   PMat = np.array( \
         [[p11, p12, p13], \
         [p21, p22, p23], \
         [p31, p32, p33] \
         ])

   #### Jacobian Matrix for Rotational Dynamics
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
   ##########

   #### Rotational Dynamics
   w1=dpio-np.sin(thetao)*dpsio
   w2=np.cos(pio)*dthetao+(np.sin(pio)*np.cos(thetao))*dpsio
   w3=-np.sin(pio)*dthetao+(np.cos(pio)*np.cos(thetao))*dpsio
   ## Disturbance for the roational dynamics
   dis_R_pi=np.cos(t)
   dis_R_theta=np.cos(t)
   dis_R_psi=np.cos(t)
   ## Dynamic inversion for Rotational Dynamics 
   f1d=(np.cos(pio)*np.tan(thetao)*w2-np.sin(pio)*np.tan(thetao)*w3)*dpio \
      +(((1/np.cos(thetao))**2*np.sin(pio))*w2+(np.cos(pio)*(1/np.cos(thetao))**2)*w3)*dthetao \
      -((izz-iyy)/ixx)*w2*w3 \
      +(np.sin(pio)*np.tan(thetao))*((izz-ixx)/iyy)*w1*w3 \
      -(np.cos(pio)*np.tan(thetao))*((iyy-ixx)/izz)*w1*w2 \
      +(1/ixx)*u2 \
      +((np.sin(pio)*np.tan(thetao))/iyy)*u3 \
      +((np.cos(pio)*np.tan(thetao))/izz)*u4
   ##########
   f2d=-(np.sin(pio)*w2+np.cos(pio)*w3)*dpio \
      +np.cos(pio)*((izz-ixx)/iyy)*w1*w3 \
      +np.sin(pio)*((iyy-ixx)/izz)*w1*w2 \
      +(np.cos(pio)/iyy)*u3 \
      -(np.sin(pio)/izz)*u4
   ##########
   f3d=(np.cos(pio)*1/np.cos(thetao)*w2-1/np.cos(thetao)*np.sin(pio)*w3)*dpio \
      +(1/np.cos(thetao)*np.sin(pio)*np.tan(thetao)*w2+np.cos(pio)*1/np.cos(thetao)*np.tan(thetao)*w3)*dthetao \
      +(1/np.cos(thetao)*np.sin(pio))*((izz-ixx)/iyy)*w1*w3 \
      -(np.cos(pio)*1/np.cos(thetao))*((iyy-ixx)/izz)*w1*w2 \
      +(1/np.cos (thetao)*np.sin(pio)/iyy)*u3 \
      +(np.cos(pio)*1/np.cos(thetao)/izz)*u4

   #### Design Extended High Gain Observer
   hw1s=hdpio-np.sin(hthetao)*hdpsios
   hw2s=np.cos(hpio)*hdthetaos+(np.sin(hpio)*np.cos(hthetao))*hdpsios
   hw3s=-np.sin(hpio)*hdthetaos+(np.cos(hpio)*np.cos(hthetao))*hdpsios
   ##########
   f1hs=(np.cos(pio)*np.tan(thetao)*hw2s-np.sin(pio)*np.tan(thetao)*hw3s)*hdpios \
      +(((1/np.cos(thetao))**2*np.sin(pio))*hw2s+(np.cos(pio)*(1/np.cos(thetao))**2)*hw3s)*hdthetaos \
      -((izz-iyy)/ixx)*hw2s*hw3s \
      +(np.sin(pio)*np.tan(thetao))*((izz-ixx)/iyy)*hw1s*hw3s \
      -(np.cos(pio)*np.tan(thetao))*((iyy-ixx)/izz)*hw1s*hw2s \
      +(1/ixx)*u2 \
      +((np.sin(pio)*np.tan(thetao))/iyy)*u3 \
      +((np.cos(pio)*np.tan(thetao))/izz)*u4
   ##########
   f2hs=-(np.sin(pio)*hw2s+np.cos(pio)*hw3s)*hdpios \
      +np.cos(pio)*((izz-ixx)/iyy)*hw1s*hw3s \
      +np.sin(pio)*((iyy-ixx)/izz)*hw1s*hw2s \
      +(np.cos(pio)/iyy)*u3 \
      -(np.sin(pio)/izz)*u4
   ##########
   f3hs=(np.cos(pio)*1/np.cos(thetao)*hw2s-1/np.cos(thetao)*np.sin(pio)*hw3s)*hdpios \
      +(1/np.cos(thetao)*np.sin(pio)*np.tan(thetao)*hw2s+np.cos(pio)*1/np.cos(thetao)*np.tan(thetao)*hw3s)*hdthetaos \
      +(1/np.cos(thetao)*np.sin(pio))*((izz-ixx)/iyy)*hw1s*hw3s \
      -(np.cos(pio)*1/np.cos(thetao))*((iyy-ixx)/izz)*hw1s*hw2s \
      +(1/np.cos(thetao)*np.sin(pio)/iyy)*u3 \
      +(np.cos(pio)*1/np.cos(thetao)/izz)*u4

   ##############################################################################
   pi_v_test = 0
   theta_v_test = 0
   psi_r_test = 0

   f1ErrorC=f1hs+dis_R_pi - (-kpr*(pio-pi_v)-kvr*dpio)
   f2ErrorC=f2hs+dis_R_theta - (-kpr*(thetao-theta_v)-kvr*dthetao)
   f3ErrorC=f3hs+dis_R_psi - (-kpr*(psio-psi_r)-kvr*dpsio)


   ########

   ###########################################################################
   #### Extended High-Gain Observers dynamics
   ###########################################################################
   hf1xn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.cos(psi_r)+np.sin(pio)*np.sin(psi_r))*u1)
   hf2yn=(1/m)*(-(np.cos(pio)*np.sin(thetao)*np.sin(psi_r)-np.sin(pio)*np.cos(psi_r))*u1)
   hf3zn=(1/m)*(-(np.cos(pio)*np.cos(thetao))*u1)+g

   # Update states
   for i in range(51):
      x_p[i] = x[i]

   # References
   x[9]  = x[9] + dt*x[12]
   x[10] = x[10] + dt*x[13]
   x[11] = x[11] + dt*x[14]
   x[12] = x_p[12] + dt*(-kp*(x_p[9]-x_r) - kv*(x_p[12]-x2_r) + x3_r)
   x[13] = x_p[13] + dt*(-kp*(x_p[10]-y_r) - kv*(x_p[13]-y2_r) + y3_r)
   # x[14] = x_p[14] + dt*(-kpz*(x_p[11]-z_r) - kvz*(x_p[14]-z2_r) + z3_r)
   x[14] = x_p[14] + dt*(-kpz*(x_p[11]-5.) - kvz*(x_p[14]-0.) + z3_r)


   # Dynamic Inversion
   fErrorMat = np.array([[f1ErrorC], [f2ErrorC], [f3ErrorC]])

   # print(fErrorMat)

   dfduMat = np.array([[dfpidu2, dfpidu3, dfpidu4], \
                  [dfthetadu2, dfthetadu3, dfthetadu4], \
                  [dfpsidu2, dfpsidu3, dfpsidu4] \
                  ])
   duMat = -(1/elip_Rotaional)*multi_dot([np.transpose(dfduMat),np.transpose(PMat),fErrorMat])
   dPMat = -1*multi_dot([PMat,np.transpose(dfduMat),dfduMat,PMat])

   # u2, u3, u4
   x[21] = x_p[21] + dt*duMat[0,0]
   x[22] = x_p[22] + dt*duMat[1,0]
   x[23] = x_p[23] + dt*duMat[2,0]

   # Low pass filter
   u2o = x[21]
   u3o = x[22]
   u4o = x[23]

   u2f = (1-a)*x_p[21] + a*u2o
   u3f = (1-a)*x_p[22] + a*u3o
   u4f = (1-a)*x_p[23] + a*u4o

   # Update values
   x[21] = u2f
   x[22] = u3f
   x[23] = u4f
   u2 = u2f
   u3 = u3f
   u4 = u4f

   
   # P Matrix
   x[42] = x_p[42] + dt*dPMat[0,0]
   x[43] = x_p[43] + dt*dPMat[0,1]
   x[44] = x_p[44] + dt*dPMat[0,2]
   x[45] = x_p[45] + dt*dPMat[1,0]
   x[46] = x_p[46] + dt*dPMat[1,1]
   x[47] = x_p[47] + dt*dPMat[1,2]
   x[48] = x_p[48] + dt*dPMat[2,0]
   x[49] = x_p[49] + dt*dPMat[2,1]
   x[50] = x_p[50] + dt*dPMat[2,2]


   ###########################################################################
   hw1=hdpio-np.sin(hthetao)*hdpsio
   hw2=np.cos(hpio)*hdthetao+(np.sin(hpio)*np.cos(hthetao))*hdpsio
   hw3=-np.sin(hpio)*hdthetao+(np.cos(hpio)*np.cos(hthetao))*hdpsio
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
   x[24]=x_p[24] + dt*(x_p[25]+(alpha11/elipHr)*(x_p[15]-x_p[24]))
   x[25]=x_p[25] + dt*(f1h+x_p[26]+(alpha12/elipHr**2)*(x_p[15]-x_p[24]))
   x[26]=x_p[26] + dt*((alpha13/elipHr**3)*(x_p[15]-x_p[24]))
   #####Measurement:  x[17]:\theteta##################################################################
   x[27]=x_p[27] + dt*(x_p[28]+(alpha21/elipHr)*(x_p[16]-x_p[27]))
   x[28]=x_p[28] + dt*(f2h+x_p[29]+(alpha22/elipHr**2)*(x_p[16]-x_p[27]))
   x[29]=x_p[29] + dt*((alpha23/elipHr**3)*(x_p[16]-x_p[27]))
   #####Measurement: x[18]:\psi####################
   x[30]=x_p[30] + dt*(x_p[31]+(alpha31/elipHr)*(x_p[17]-x_p[30]))
   x[31]=x_p[31] + dt*(f3h+x_p[32]+(alpha32/elipHr**2)*(x_p[17]-x_p[30]))
   x[32]=x_p[32] + dt*(alpha33/elipHr**3)*(x_p[17]-x_p[30])

   #######Design of Extended High-Gain Observers for Translational dynamics########
   x[33]=x_p[33]+dt*(x_p[34]+(alpha11p/elipHt)*(x_p[0]-x_p[33]))
   x[34]=x_p[34]+dt*(hf1xn+x_p[35]+(alpha21p/elipHt**2)*(x_p[0]-x_p[33]))
   x[35]=x_p[35]+dt*((alpha31p/elipHt**3)*(x_p[0]-x_p[33]))
   ###### Y-dynamics#################################
   x[36]=x_p[36]+dt*(x_p[37]+(alpha11p/elipHt)*(x_p[1]-x_p[36]))
   x[37]=x_p[37]+dt*(hf2yn+x_p[38]+(alpha21p/elipHt**2)*(x_p[1]-x_p[36]))
   x[38]=x_p[38]+dt*(alpha31p/elipHt**3)*(x_p[1]-x_p[36])
   ######## Z-dynamics ###############################
   x[39]=x_p[39]+dt*(x_p[40]+(beta11p/elipHt)*(x_p[2]-x_p[39]))
   x[40]=x_p[40]+dt*(hf3zn+x_p[41]+(beta21p/elipHt**2)*(x_p[2]-x_p[39]))
   x[41]=x_p[41]+dt*((beta31p/elipHt**3)*(x_p[2]-x_p[39]))
   ########################################


   # Convert to Forces
   d1 = 0.1572
   d2 = 0.1572
   c = 0.1

   # f1 = (u1/4 - u2/(4*d2) + u3/(4*d1) + u4/(4*c))
   # f2 = (u1/4 + u2/(4*d2) - u3/(4*d1) + u4/(4*c))
   # f3 = (u1/4 + u2/(4*d2) + u3/(4*d1) - u4/(4*c))
   # f4 = (u1/4 - u2/(4*d2) - u3/(4*d1) - u4/(4*c))

   M1 = (u1 - u2 + u3 + u4)
   M2 = (u1 + u2 - u3 + u4)
   M3 = (u1 + u2 + u3 - u4)
   M4 = (u1 - u2 - u3 - u4)


   # p = [2.2685/1000000, -0.0035, 2.5983, 967.6192]
    
   # M1 = p[0]*f1**3 + p[1]*f1**2 + p[2]*f1 + p[3]
   # M2 = p[0]*f2**3 + p[1]*f2**2 + p[2]*f2 + p[3]
   # M3 = p[0]*f3**3 + p[1]*f3**2 + p[2]*f3 + p[3]
   # M4 = p[0]*f4**3 + p[1]*f4**2 + p[2]*f4 + p[3]


   # n1 = np.sqrt(f1/(C_T*D^4*rho))*2*np.pi
   # n2 = np.sqrt(f2/(C_T*D^4*rho))
   # n3 = np.sqrt(f3/(C_T*D^4*rho))
   # n4 = np.sqrt(f4/(C_T*D^4*rho))

   # p = [-0.0044, 9.2052, 1613.]
   # mul = 4.8
   # M1 = (p[0]*f1**2 + p[1]*f1 + p[2])/mul 
   # M2 = (p[0]*f2**2 + p[1]*f2 + p[2])/mul
   # M3 = (p[0]*f3**2 + p[1]*f3 + p[2])/mul
   # M4 = (p[0]*f4**2 + p[1]*f4 + p[2])/mul
   # mul = 200.
   # add = 410.
   # M1 = f1 / mul + add
   # M2 = f2 / mul + add
   # M3 = f3 / mul + add
   # M4 = f4 / mul + add

   rotors = np.clip([M1,M2,M3,M4],-5000,5000.)
   forces = [M1,M2,M3,M4]

   return x, rotors, forces, pi_v