import numpy as np
from numpy.linalg import multi_dot


class RLS_EHGO_T:
      def __init__(self, general_settings, settings):
            self.T = general_settings[0]
            self.g = general_settings[1]
            self.m = general_settings[2]

            # Rotational EHGO
            self.epsilon = settings[0]
            self.a10 = settings[1]
            self.a11 = settings[2]
            self.a12 = settings[3]
            self.b10 = settings[4]
            self.b11 = settings[5]
            self.b12 = settings[6]

      def EHGO_T(self, EHGO_T_input):
            s_ = EHGO_T_input

            # Inputs for EHGO
            U1f_ = s_[0]
            phi_ = s_[1]
            theta_ = s_[2]
            psi_ = s_[3]
            x2_ = s_[4]
            y2_ = s_[5]
            z2_ = s_[6]
            x2_ob_ = s_[7]
            y2_ob_ = s_[8]
            z2_ob_ = s_[9]
            sig_x_ = s_[10]
            sig_y_ = s_[11]
            sig_z_ = s_[12]

            # Output for EHGO
            # sig_phi 
            # sig_theta 
            # sig_psi 
            # phi2_ob 
            # theta2_ob 
            # psi2_ob 


            # Dynamics
            Fx=(1/self.m)*(-(np.cos(phi_)*np.sin(theta_)*np.cos(psi_)+np.sin(phi_)*np.sin(psi_))*U1f_)
            Fy=(1/self.m)*(-(np.cos(phi_)*np.sin(theta_)*np.sin(psi_)-np.sin(phi_)*np.cos(psi_))*U1f_)
            Fz=(1/self.m)*(-(np.cos(phi_)*np.cos(theta_))*U1f_)+self.g

            x2_ob = x2_ob_ + self.T*(Fx + sig_x_ + self.a11/self.epsilon*(x2_ - x2_ob_))
            y2_ob = y2_ob_ + self.T*(Fy + sig_y_ + self.a11/self.epsilon*(y2_ - y2_ob_))
            z2_ob = z2_ob_ + self.T*(Fz + sig_z_ + self.b11/self.epsilon*(z2_ - z2_ob_))

            sig_x = sig_x_ + self.T*(self.a12/self.epsilon**2*(x2_ - x2_ob_))
            sig_y = sig_y_ + self.T*(self.a12/self.epsilon**2*(y2_ - y2_ob_))
            sig_z = sig_z_ + self.T*(self.b12/self.epsilon**2*(z2_ - z2_ob_))

            s = [x2_ob, y2_ob, z2_ob, sig_x, sig_y, sig_z]

            return s