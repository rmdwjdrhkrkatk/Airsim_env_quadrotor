import numpy as np
from numpy.linalg import multi_dot


class RLS_C_T:
      def __init__(self, general_settings, settings):
            self.T = general_settings[0]
            self.g = general_settings[1]
            self.m = general_settings[2]
            self.a = general_settings[6]

            self.kx1 = settings[0]
            self.kx2 = settings[1]

            self.ky1 = settings[2]
            self.ky2 = settings[3]
            
            self.kz1 = settings[4]
            self.kz2 = settings[5]


      def controller_translation(self, T, translational_controller_input):
            ########################### Current States #############################
            tc_ = translational_controller_input

            # Inputs
            x_r = tc_[0]
            y_r = tc_[1]
            z_r = tc_[2]

            x2_r = tc_[3]
            y2_r = tc_[4]
            z2_r = tc_[5]

            x3_r = tc_[6]
            y3_r = tc_[7]
            z3_r = tc_[8]

            x_ = tc_[9]
            y_ = tc_[10]
            z_ = tc_[11]

            x2_ = tc_[12]
            y2_ = tc_[13]
            z2_ = tc_[14]

            sig_x_ = tc_[15]
            sig_y_ = tc_[16]
            sig_z_ = tc_[17]

            phi_ = tc_[18]
            theta_ = tc_[19]
            psi_ = tc_[20]

            U1f_ = tc_[21]

            # Outputs
            # phi_r
            # theta_r
            # U1
            # U1f

            ########################### Rotational Control ############################
            ## Dynamic inversion for Rotational Dynamics 
            frx = -self.kx1*(x_ - x_r) - self.kx2*(x2_ - x2_r) - sig_x_
            fry = -self.ky1*(y_ - y_r) - self.ky2*(y2_ - y2_r) - sig_y_
            frz = -self.kz1*(z_ - z_r) - self.kz2*(z2_ - z2_r) - sig_z_

            theta_r = np.arctan((np.sin(psi_)*fry/(frz - self.g)) + (np.cos(psi_)*frx/(frz - self.g)))
            theta_r = min(abs(theta_r),0.6)*np.sign(theta_r)
            phi_r = np.arctan((np.cos(theta_)*np.sin(psi_)*frx/(frz - self.g)) - (np.cos(psi_)*np.cos(theta_)*fry/(frz - self.g)))
            phi_r = min(abs(phi_r),0.6)*np.sign(phi_r)

            U1 = -(self.m*(frz - self.g))/(np.cos(theta_)*np.cos(phi_))

            U1f = self.a*U1 + (1-self.a)*U1f_

            output = [phi_r, theta_r, U1, U1f]

            return output