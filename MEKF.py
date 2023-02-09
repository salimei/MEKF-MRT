# -*- coding: utf-8 -*-
""" 

EKF additional features: 
            -Jacobian instead of transition matrix 

There are many sensors aboard the rocket, the current string format is: 
S,ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,PRESSURE,LAT,LONG,MIN,SEC,SUBSEC,STATE,CONT,E
where S: Start, E: end, CONT: continuity (pyro). 

However, we only care about:
    -IMU: ACCx,ACCy,ACCz,GYROx,GYROy,GYROz
    -GPS: LAT,LONG
    -Barometer: PRESSURE
    -Time: MIN,SEC,SUBSEC (optional)

"""

import numpy as np
#from numpy.linalg import inv
from scipy.linalg import inv
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm, sinm, cosm

class MEKF:

    """ process model
            Cab_k = Cab_k-1 . e^(T*gyro_input)
            Va_k = Va_k-1 + T . Cab_k-1 . acc_input + T . ga
            ra_k = ra_k-1 + Va_k-1 . T
            P_k = A_k-1 . P_k-1 . (A_k-1)^T + L_k-1 . Q_k-1 . (L_k-1)


            P_k = (1 - K_k . C_k ) . P_k . (1 - K_k . C_k)^T + K_k . M_k . R_k . (M_k)^T . (K_k)^T 
            K_k = P_k . (C_k)^T (C_k . P_k . (C_k)^T + M_k . R_k . (M_k)^T )^-1
            correction_term = K_k(GPS_input - ra_k)

            Cab_k = Cab_k . e^(-correction_term)
            Va_k = Va_k + correction_term
            ra_k = r_ak + correction_term
    """

    
    def __init__(self, dt, sigma_gyro, sigma_acc, sigma_gps, P_init_orien, P_init_pos):
        #dummy variables
        self.zeros3 = np.zeros((3, 3), dtype='f')
        self.ones3 = np.eye((3), dtype='f')
        
        #predict 
        euler_Cab = np.array([0, 0, 0])
        self.Cab_k_1 = R.from_euler('zyx', euler_Cab, degrees=False).as_matrix() # rotation matrix (DCM at k-1)
        self.Va_k_1 = np.zeros((3, 1), dtype='f').T # initial speed
        self.ra_k_1 = np.zeros((3, 1), dtype='f').T # initial position
        self.ga = np.array([0, 0, -9.81], dtype='f').T # gravitational constant

            #A and B
        self.A = np.zeros((9, 9), dtype='f') #state transition model (process model)
        self.T = dt #state transition model (process model) (B = T)

            #noise 
            #covariance of process noise (error on prediction)
        cov_gyro = sigma_gyro**2
        cov_acc = sigma_acc**2
        cov_gps = sigma_gps**2
        self.Q = np.block([[self.ones3*cov_gyro,self.zeros3],
                          [self.zeros3, self.ones3*cov_acc]])
        
        
        self.R = np.eye(3, dtype='f')*cov_gps #covariance of obervation noise (sensor noise)
        # Q : get value when robot is static 
        # R : get value from sensor datasheet 

            #sensor intial
        self.GYRO_input = np.zeros((3, 1), dtype='f') #initial input (GYRO)
        self.ACC_input = np.zeros((3, 1), dtype='f') #initial input (IMU)

            #other variables
        self.Cab_k = np.eye(3, dtype='f')
        self.Va_k = np.zeros((3, 1), dtype='f')
        self.ra_k = np.zeros((3, 1), dtype='f')
        self.P_k = np.eye(9, dtype='f') 
        self.P_k_1 =  np.block([[self.ones3*P_init_orien,    self.zeros3 ,      self.zeros3],
                                [self.zeros3,           self.ones3*P_init_pos,   self.zeros3], 
                                [self.zeros3,           self.zeros3,        self.ones3*P_init_pos]])
        self.L = np.zeros((9, 6), dtype='f')
        self.S1 = np.eye(9, dtype='f') 
        self.S2 = np.eye(3, dtype='f')

        #correct
        self.correction_term = np.zeros((9, 1), dtype='f')
        self.K_k = np.zeros((9, 3), dtype='f')
        
            #sensor 
        self.GPS_input =  np.zeros((3, 1), dtype='f')

        self.M_k = np.eye(3, dtype='f')
        self.C_k = np.block([self.zeros3, self.zeros3, self.ones3])

        print("init done")
        
    def kf_predict(self, GYRO_input, ACC_input): 
        #convert euler gyro into rotation matrix gyro
        c = GYRO_input.T
        gyro_cross = np.array([[0, -c[2], c[1]],
                             [c[2], 0, -c[0]],
                             [-c[1], c[0], 0]])
        c = ACC_input.T
        acc_cross = np.array([[0, -c[2], c[1]],
                             [c[2], 0, -c[0]],
                             [-c[1], c[0], 0]])
        self.ACC_input = c
        
        self.Cab_k = self.Cab_k_1 @ expm(self.T*gyro_cross)
        
        self.Va_k = self.Va_k_1 + self.T * self.Cab_k_1 @ self.ACC_input + self.T * self.ga
        self.ra_k = self.ra_k_1 + self.Va_k_1 * self.T
        "failure point, array probably [[1, 2, 3], [4, 5, 6]]"
        self.A = np.block( [[(expm(self.T * gyro_cross)).T,       self.zeros3 , self.zeros3],
                            [self.T * self.Cab_k_1 @ acc_cross, self.ones3,   self.zeros3], 
                            [self.zeros3,              np.eye(3) *   self.T,       self.ones3]])
        self.L = np.block([[np.eye(3)*self.T, self.zeros3],
                          [self.zeros3, -self.T*self.Cab_k_1],
                          [self.zeros3, self.zeros3]])
        self.P_k = self.A @ self.P_k_1 @ self.A.T + self.L @ self.Q @ self.L.T

    def kf_correct(self, GPS_input):
        self.GPS_input = GPS_input.T 
        self.S2 = self.M_k @ self.R @ self.M_k.T #utility
        self.K_k = self.P_k @ self.C_k.T @ inv(self.C_k @ self.P_k @ self.C_k.T + self.S2)

        self.S1 = np.eye(9) - self.K_k @ self.C_k #utility
        self.P_k = self.S1 @ self.P_k @ self.S1.T + self.K_k @ self.S2 @ self.K_k.T
        self.correction_term = self.K_k @ (self.GPS_input - self.ra_k.T)

        r = np.block(self.correction_term[0:3])
        correct_cross = np.array([[0, -r[2], r[1]],
                             [r[2], 0, -r[0]],
                             [-r[1], r[0], 0]], dtype='f')
        self.Cab_k = self.Cab_k @ expm(-correct_cross) #fa
        self.Va_k = self.Va_k +  self.correction_term[3:6].T
        self.ra_k = self.ra_k + self.correction_term[6:9].T
    
    #shuffles all k to k-1. ex: x[k-1] = x[k]
    def kf_update(self):
        self.Cab_k_1 = self.Cab_k
        self.Va_k_1 = self.Va_k
        self.ra_k_1 = self.ra_k
        self.P_k_1 = self.P_k
        




