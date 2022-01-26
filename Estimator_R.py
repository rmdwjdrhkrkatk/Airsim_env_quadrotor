import numpy as np
from numpy.linalg import multi_dot


class RLS_EHGO_R:
      def __init__(self, general_settings, settings):
            # mass = 1.
            # motor_assembly_wight = 0.055
            # box_mass = 1 - 0.055*4 = 0.78

            # body_box.x = 0.18
            # body_box.y = 0.11
            # body_box.z = 0.04
            # rotor_z = 0.025

            self.T = general_settings[0]

            self.Ixx = general_settings[3]
            self.Iyy = general_settings[4]
            self.Izz = general_settings[5]
            
            # EHGOs
            # Rotational EHGO
            self.epsilonr = settings[0]
            self.r1 = settings[1]
            self.r2 = settings[2]
            self.r3 = settings[3]

            # 0 U2 U3 U4 2
            # 3 phi theta psi 5
            # 6 phi2 theta2 psi2 8
            # 9 phi2_ob theta2_ob psi_ob 11
            # 12 sig_phi sig_theta sig_psi 14

      def EHGO_R(self, T, EHGO_R_input):
            s_ = EHGO_R_input

            # Inputs for EHGO
            U2f_ = s_[0]
            U3f_ = s_[1]
            U4f_ = s_[2]
            phi_ = s_[3]
            theta_ = s_[4]
            psi_ = s_[5]
            phi2_ = s_[6]
            theta2_ = s_[7]
            psi2_ = s_[8]
            phi2_ob_ = s_[9]
            theta2_ob_ = s_[10]
            psi2_ob_ = s_[11]
            sig_phi_ = s_[12]
            sig_theta_ = s_[13]
            sig_psi_ = s_[14]


            # Output for EHGO
            # sig_phi 
            # sig_theta 
            # sig_psi 
            # phi2_ob 
            # theta2_ob 
            # psi2_ob 


            # Dynamics
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

            phi2_ob = phi2_ob_ + T*(-Ac[0] + Bc[0] + Mdc[0] + sig_phi_ + self.r1/self.epsilonr*(phi2_ - phi2_ob_))
            theta2_ob = theta2_ob_ + T*(-Ac[1] + Bc[1] + Mdc[1] + sig_theta_ + self.r1/self.epsilonr*(theta2_ - theta2_ob_))
            psi2_ob = psi2_ob_ + T*(-Ac[2] + Bc[2] + Mdc[2] + sig_psi_ + self.r1/self.epsilonr*(psi2_ - psi2_ob_))

            sig_phi = sig_phi_ + T*(self.r2/self.epsilonr**2*(phi2_ - phi2_ob_))    
            sig_theta = sig_theta_ + T*(self.r2/self.epsilonr**2*(theta2_ - theta2_ob_))
            sig_psi = sig_psi_ + T*(self.r2/self.epsilonr**2*(psi2_ - psi2_ob_))

            s = [phi2_ob, theta2_ob, psi2_ob, sig_phi, sig_theta, sig_phi]

            return s