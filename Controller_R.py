import numpy as np
from numpy.linalg import multi_dot


class RLS_C_R:
      def __init__(self, general_settings, settings):
            ### Success List
            ## 1
            # self.elip_Rotaional=0.000003
            # self.PInit = 0.00000005
            # dtf = 0.55

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

            

            # mass = 1.
            # motor_assembly_wight = 0.055
            # box_mass = 1 - 0.055*4 = 0.78

            # body_box.x = 0.18
            # body_box.y = 0.11
            # body_box.z = 0.04
            # rotor_z = 0.025

            

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


      def controller_rotation(self, T, rotational_controller_input):
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
            psi_r = 0.
            Fr = [-self.kpr*(phi_ - phi_v) - self.kvr*(phi2_),\
                  -self.kpr*(theta_ - theta_v) - self.kvr*(theta2_),\
                  -self.kpr*(psi2_ - psi_r)] # - self.kvr*(psi2_)]

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


            f1ErrorC = -Ac[0] + Bc[0] + Mdc[0] - Fr[0] - sig_phi_
            f2ErrorC = -Ac[1] + Bc[1] + Mdc[1] - Fr[1] - sig_phi_
            f3ErrorC = -Ac[2] + Bc[2] + Mdc[2] - Fr[2] - sig_theta_

            fErrorMat = [f1ErrorC, f2ErrorC, f3ErrorC]
            # P Matrix
            PMat = np.array([[p11_, p12_, p13_], \
                  [p21_, p22_, p23_], \
                  [p31_, p32_, p33_] \
                  ])
            #######Jacobian Matrix for Rotational Dynamics############################
            dfpidu2 = 1./self.Ixx
            dfpidu3 = (np.sin(phi_)*np.tan(theta_))/self.Iyy
            dfpidu4 = (np.cos(phi_)*np.tan(theta_))/self.Iyy
            ##########
            dfthetadu2 = 0.
            dfthetadu3 = np.cos(phi_)/self.Iyy
            dfthetadu4 = - (np.sin(phi_)/self.Iyy)
            ##########
            dfpsidu2 = 0.
            dfpsidu3 = np.sin(phi_)/self.Iyy/np.cos(theta_)
            dfpsidu4 = np.cos(phi_)/np.cos(theta_)/self.Iyy

            dfduMat = np.array([[dfpidu2, dfpidu3, dfpidu4], \
                  [dfthetadu2, dfthetadu3, dfthetadu4], \
                  [dfpsidu2, dfpsidu3, dfpsidu4] \
                  ])


            # duMat = -(1/elip_Rotaional)*multi_dot([np.transpose(dfduMat),np.transpose(PMat),fErrorMat])
            # dPMat = -1*multi_dot([PMat,np.transpose(dfduMat),dfduMat,PMat])

            duMat = -(1/self.elip_Rotaional)*multi_dot([np.transpose(dfduMat),np.transpose(PMat),fErrorMat])
            dPMat = -1*multi_dot([PMat,np.transpose(dfduMat),dfduMat,PMat])

            # P Matrix
            p11 = p11_ + T*dPMat[0,0]
            p12 = p12_ + T*dPMat[0,1]
            p13 = p13_ + T*dPMat[0,2]
            p21 = p21_ + T*dPMat[1,0]
            p22 = p22_ + T*dPMat[1,1]
            p23 = p23_ + T*dPMat[1,2]
            p31 = p31_ + T*dPMat[2,0]
            p32 = p32_ + T*dPMat[2,1]
            p33 = p33_ + T*dPMat[2,2]

            # U2, U3, U4
            U2 = U2f_ + T*duMat[0]
            U3 = U3f_ + T*duMat[1]
            U4 = U4f_ + T*duMat[2]

            U2f = (1-self.a)*U2f_ + self.a*U2
            U3f = (1-self.a)*U3f_ + self.a*U3
            U4f = (1-self.a)*U4f_ + self.a*U4

            output = [U2f, U3f, U4f, U2, U3, U4, p11, p12, p13, p21, p22, p23, p31, p32, p33]

            return output