import numpy as np
from numpy.linalg import multi_dot

import rospy
from sensor_msgs.msg import Imu

class RLS_C_R:
      def __init__(self, general_settings, settings):
                       

            self.T = general_settings[0]
            self.g = general_settings[1]
            self.m = general_settings[2]
            self.Ixx = general_settings[3]
            self.Iyy = general_settings[4]
            self.Izz = general_settings[5]
            self.a = general_settings[6]

            self.elip_Rotaional=settings[0]
            self.kpr = settings[1]
            self.kvr = settings[2]


      def controller_rotation(self,rotational_controller_input):
            ########################### Current States #############################
            rc_ = rotational_controller_input

            # Inputs
            phi_v = rc_[0]
            theta_v = rc_[1]
            psi_r = rc_[2]

            phi_ = rc_[3]
            theta_ = rc_[4]
            psi_ = rc_[5]

            phi2_ = rc_[6]
            theta2_ = rc_[7]
            psi2_ = rc_[8]

            sig_phi_ = rc_[9]
            sig_theta_ = rc_[10]
            sig_psi_ = rc_[11]

            U2f_ = rc_[12]
            U3f_ = rc_[13]
            U4f_ = rc_[14]
            p11_ = rc_[15]
            p12_ = rc_[16]
            p13_ = rc_[17]
            p21_ = rc_[18]
            p22_ = rc_[19]
            p23_ = rc_[20]
            p31_ = rc_[21]
            p32_ = rc_[22]
            p33_ = rc_[23]

            ########################### Rotational Control ############################
            ## Dynamic inversion for Rotational Dynamics 

            Fr = np.transpose([-self.kpr*(phi_ - phi_v) - self.kvr*(phi2_),\
                  -self.kpr*(theta_ - theta_v) - self.kvr*(theta2_),\
                  -self.kpr*(psi_ - psi_r) - self.kvr*(psi2_)])

            wc = [phi2_ - psi2_ * np.sin(theta_),\
                  theta2_*np.cos(phi_)+psi2_*(np.sin(phi_)*np.cos(theta_)),\
                  -theta2_*np.sin(phi_)+psi2_*(np.cos(phi_)*np.cos(theta_))]
                  
            Ac = [((self.Izz-self.Iyy)/self.Ixx)*wc[1]*wc[2]+((self.Ixx-self.Izz)/self.Iyy)*wc[0]*wc[2]*np.sin(phi_)*np.tan(theta_)+((self.Iyy-self.Ixx)/self.Izz)*wc[0]*wc[1]*np.cos(phi_)*np.tan(theta_),\
                  ((self.Ixx-self.Izz)/self.Iyy)*wc[0]*wc[2]*np.cos(phi_)-((self.Iyy-self.Ixx)/self.Izz)*wc[0]*wc[1]*np.sin(phi_),\
                  ((self.Ixx-self.Izz)/self.Iyy)*wc[0]*wc[2]*np.sin(phi_)/np.cos(theta_)+((self.Iyy-self.Ixx)/self.Izz)*wc[0]*wc[1]*np.cos(phi_)/np.cos(theta_)]
                  
            Mdc = [(phi2_*np.cos(phi_)*np.tan(theta_)+theta2_*np.sin(phi_)/pow(np.cos(theta_),2))*wc[1] + (-phi2_*np.sin(phi_)*np.tan(theta_)+theta2_*np.cos(phi_)/pow(np.cos(theta_),2))*wc[2],\
                  (-phi2_*np.sin(phi_))*wc[1]+(-phi2_*np.cos(phi_))*wc[2],\
                  (phi2_*np.cos(phi_)/np.cos(theta_)+theta2_*np.sin(phi_)/np.cos(theta_)*np.cos(theta_)*np.tan(theta_))*wc[1]+(-phi2_*np.sin(phi_)/np.cos(theta_)+theta2_*np.cos(phi_)/np.cos(theta_)*np.tan(theta_))*wc[2]]   

            Bc = [(U2f_/self.Ixx)+(np.sin(phi_)*np.tan(theta_)/(self.Iyy))*U3f_+(np.cos(phi_)*np.tan(theta_)/(self.Izz))*U4f_,\
                  (np.cos(phi_)*U3f_/(self.Iyy))-(np.sin(phi_)*U4f_/(self.Izz)),\
                  (np.sin(phi_)/np.cos(theta_)*U3f_/(self.Iyy))+(np.cos(phi_)/np.cos(theta_)*U4f_/(self.Izz))]



            G = np.array([[1/self.Ixx, np.sin(phi_)*np.tan(theta_)/self.Iyy, np.cos(phi_)*np.tan(theta_)/self.Izz],\
                        [0, np.cos(phi_)/self.Iyy, -np.sin(phi_)/self.Izz],
                        [0, np.sin(phi_)/np.cos(theta_)/self.Iyy, np.cos(phi_)/np.cos(theta_)/self.Izz]])
            F = np.transpose([-Ac[0] + Mdc[0],-Ac[1] + Mdc[1],-Ac[2] + Mdc[2]])

            U = multi_dot([np.linalg.inv(G),(-F+Fr)])

            # U2, U3, U4
            U2 = U[0]
            U3 = U[1]
            U4 = U[2]

            a = 0.1
            U2f = (1-a)*U2f_ + a*U2
            U3f = (1-a)*U3f_ + a*U3
            U4f = (1-a)*U4f_ + a*U4

            output = [U2f, U3f, U4f, U2, U3, U4, 0, 0, 0, 0, 0, 0, 0, 0, 0]

            return output