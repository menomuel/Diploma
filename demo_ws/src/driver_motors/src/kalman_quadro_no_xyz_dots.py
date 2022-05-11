# coding: utf-8

import math
from numpy import *
import sys
import os
os.environ["OMP_NUM_THREADS"] = "1"

try:
    import core._dotblas
    print ('FAST BLAS')
except ImportError:
    print ('slow blas')

print ("numpy version:", __version__)

from random import gauss
from libstd import angle_to_pi

#from libstd import fetch_angle_to_2pi
PRINT = 0



class Kalman:
    def __init__(self, x0=0., y0=0.):        

#indoor
        
        self.R = array([[0.0001,   0.,  0.,   0.,  0.,  0.,   0.,  0.,  0.],   # initialize 2-d array
                        [0.,   0.0001,  0.,   0.,  0.,  0.,   0.,  0.,  0.],
                        [0.,   0.,  0.0002,   0.,  0.,  0.,   0.,  0.,  0.],
                        [0.,   0.,  0.,   0.0001,  0.,  0.,   0.,  0.,  0.],
                        [0.,   0.,  0.,   0.,  0.0001,  0.,   0.,  0.,  0.],
                        [0.,   0.,  0.,   0.,  0.,  0.0001,   0.,  0.,  0.],
                        [0.,   0.,  0.,   0.,  0.,    0.,   0.01,  0.,  0.],
                        [0.,   0.,  0.,   0.,  0.,    0.,   0.,  0.01,  0.],
                        [0.,   0.,  0.,   0.,  0.,    0.,   0.,  0.,  0.01]], dtype = double)
        

#        self.R = array([[0.01,   0.,  0.,   0.,  0.,  0.,   0.,  0.,  0.],   # initialize 2-d array
#                        [0.,   0.01,  0.,   0.,  0.,  0.,   0.,  0.,  0.],
#                        [0.,   0.,  0.01,   0.,  0.,  0.,   0.,  0.,  0.],
#                        [0.,   0.,  0.,   0.01,  0.,  0.,   0.,  0.,  0.],
#                        [0.,   0.,  0.,   0.,  0.01,  0.,   0.,  0.,  0.],
#                        [0.,   0.,  0.,   0.,  0.,  0.01,   0.,  0.,  0.],
#                        [0.,   0.,  0.,   0.,  0.,    0.,   0.1,  0.,  0.],
#                        [0.,   0.,  0.,   0.,  0.,    0.,   0.,  0.1,  0.],
#                        [0.,   0.,  0.,   0.,  0.,    0.,   0.,  0.,  0.1]], dtype = double)


#outdoor
#        self.R = array([[1.0,   0.,  0.,   0.,  0.,  0.,   0.,  0.,  0.],   # initialize 2-d array
#                        [0.,   1.0,  0.,   0.,  0.,  0.,   0.,  0.,  0.],
#                        [0.,   0.,  1.0,   0.,  0.,  0.,   0.,  0.,  0.],
#                        [0.,   0.,  0.,   0.01,  0.,  0.,   0.,  0.,  0.],
#                        [0.,   0.,  0.,   0.,  0.01,  0.,   0.,  0.,  0.],
#                        [0.,   0.,  0.,   0.,  0.,  0.01,   0.,  0.,  0.],
#                        [0.,   0.,  0.,   0.,  0.,    0.,   0.1,  0.,  0.],
#                        [0.,   0.,  0.,   0.,  0.,    0.,   0.,  0.1,  0.],
#                        [0.,   0.,  0.,   0.,  0.,    0.,   0.,  0.,  0.1]], dtype = double)

        '''
        self.R = 0.01 * eye(12)
        self.R[0][0] = 1e-4
        self.R[1][1] = 1e-4
        self.R[2][2] = 2e-4        

        self.R[3][3] = 1e-2
        self.R[4][4] = 1e-2
        self.R[5][5] = 2e-2

        self.R[6][6] = 1e-4
        self.R[7][7] = 1e-4
        self.R[8][8] = 1e-4

        self.R[9][9] = 1e-2
        self.R[10][10] = 1e-2
        self.R[11][11] = 1e-2
        '''

	# Q matrix
        self.Q = 0.00001 * eye(12) # 0.001

	# Identity
        self.I = eye(12)


    def step(self, x_estimated, covariances, x_current, x_prediction, control, params, delta_time):


        k1_psi, k1_phi, k1_theta, k2_psi, k2_phi, k2_theta, m = params

# ШАГ 1 - PREDICTION


        x_dot, y_dot, z_dot = x_estimated[3], x_estimated[4], x_estimated[5]

        psi, phi, theta = x_estimated[6], x_estimated[7], x_estimated[8]
#        psi, phi, theta = 0., 0., 0.
        




        u1, u2, u3, u4 = control
#        u1 = 4.

#        F = array([[1.,   0.,  0., delta_time,   0.,  0., 0.,   0.,  0., 0.,   0.,  0.,  0.],
#                   [0.,   1.,  0., 0.,   delta_time,  0., 0.,   0.,  0., 0.,   0.,  0.,  0.],
#                   [0.,   0.,  1., 0.,   0.,  delta_time, 0.,   0.,  0., 0.,   0.,  0.,  0.],
#                   [0.,   0.,  0., 1.,   0.,  0., u1*delta_time*time_delay*(cos(psi)*sin(phi) - sin(psi)*cos(phi)*sin(theta)), u1*delta_time*time_delay*(sin(psi)*cos(phi) - cos(psi)*sin(phi)*sin(theta)), u1*delta_time*time_delay*(cos(psi)*cos(phi)*cos(theta)), 0., 0., 0.,  u1*delta_time*(sin(psi)*sin(phi)+cos(psi)*cos(phi))],   # initialize 2-d array
#                   [0.,   0.,  0., 0.,   1.,  0., u1*delta_time*time_delay*(sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta)), u1*delta_time*time_delay*(-cos(psi)*cos(phi) - sin(psi)*sin(phi)*sin(theta)), u1*delta_time*time_delay*sin(psi)*cos(phi)*cos(theta),  0., 0., 0.,  u1*delta_time*(-cos(psi)*sin(phi)+ sin(psi)*cos(phi)*sin(theta))],
#                   [0.,   0.,  0., 0.,   0.,  1., 0., u1*delta_time*time_delay*(-sin(phi)*cos(theta)), u1*delta_time*time_delay*(-cos(phi)*sin(theta)),  0., 0., 0.,  delta_time*(u1*cos(phi)*cos(theta) - 5.)],
#                   [0.,   0.,  0., 0.,   0.,  0., 1., 0., 0., delta_time, 0., 0.,  0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0., 1., 0., 0., delta_time, 0.,  0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0., 0., 1., 0., 0., delta_time,  0.],
#                   [0.,   0.,  0., 0.,   0.,  0., -k2_psi*delta_time, 0., 0., 1.-k1_psi*delta_time, 0., 0.,  0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0., -k2_phi*delta_time, 0., 0., 1.-k1_phi*delta_time, 0.,  0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0., 0., -k2_theta*delta_time, 0., 0., 1.-k1_theta*delta_time,  0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0., 0., 0., 0., 0., 0.,  1.]
#], dtype = double)


                  # x     y    z   x_d   y_d  z_d 

        F = array([[1.,   0.,  0., delta_time,    0., 0.,  0., 0., 0., 0.,  0.,  0.],
                   [0.,   1.,  0., 0.,   delta_time,  0., 0.,  0., 0., 0.,  0.,  0.],
                   [0.,   0.,  1., 0.,   0.,  delta_time, 0.,  0., 0., 0.,  0.,  0.],
                   [0.,   0.,  0., 1.,   0.,  0., u1/m*delta_time*(cos(psi)*sin(phi) - sin(psi)*cos(phi)*sin(theta)), u1/m*delta_time*(sin(psi)*cos(phi) - cos(psi)*sin(phi)*sin(theta)), u1/m*delta_time*(cos(psi)*cos(phi)*cos(theta)), 0., 0., 0.],
                   [0.,   0.,  0., 0.,   1.,  0., u1/m*delta_time*(sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta)), u1/m*delta_time*(-cos(psi)*cos(phi) - sin(psi)*sin(phi)*sin(theta)), u1/m*delta_time*sin(psi)*cos(phi)*cos(theta),  0., 0., 0.],
                   [0.,   0.,  0., 0.,   0.,  1., 0., u1/m*delta_time*(-sin(phi)*cos(theta)), u1/m*delta_time*(-cos(phi)*sin(theta)),  0., 0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 1., 0., 0., delta_time, 0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0., 1., 0., 0., delta_time, 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0., 0., 1., 0., 0., delta_time],
                   [0.,   0.,  0., 0.,   0.,  0., -k2_psi*delta_time*1., 0., 0., 1.-k1_psi*delta_time, 0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0., -k2_phi*delta_time*1., 0., 0., 1.-k1_phi*delta_time, 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0., 0., -k2_theta*delta_time*1., 0., 0., 1.-k1_theta*delta_time]
], dtype = double)
        '''
        # MY EXPERIMENT (FEDOT)
        g = 9.81
        F = array([[1.,   0.,  0., delta_time,    0., 0.,  0., 0., 0., 0.,  0.,  0.],
                   [0.,   1.,  0., 0.,   delta_time,  0., 0.,  0., 0., 0.,  0.,  0.],
                   [0.,   0.,  1., 0.,   0.,  delta_time, 0.,  0., 0., 0.,  0.,  0.],
                   [0.,   0.,  0., 1.,   0.,  0., 0., 0., 0., 0., 0., 0.],
                   [0.,   0.,  0., 0.,   1.,  0., 0., 0., 0.,  0., 0., 0.],
                   [0.,   0.,  0., 0.,   0.,  1., 0., u1/m*delta_time*(-sin(phi)*cos(theta)), u1/m*delta_time*(-cos(phi)*sin(theta)),  0., 0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 1., 0., 0., delta_time, 0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0., 1., 0., 0., delta_time, 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0., 0., 1., 0., 0., delta_time],
                   [0.,   0.,  0., 0.,   0.,  0., -k2_psi*delta_time*1., 0., 0., 1.-k1_psi*delta_time, 0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0., -k2_phi*delta_time*1., 0., 0., 1.-k1_phi*delta_time, 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0., 0., -k2_theta*delta_time*1., 0., 0., 1.-k1_theta*delta_time]
], dtype = double)

      Bu = array([[0.],
                 [0.],
                 [0.],
                 [u1/m*delta_time*(sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta))],
                 [u1/m*delta_time*(-cos(psi)*sin(phi) + sin(psi)*cos(phi)*sin(theta))],
                 [u1/m*delta_time*cos(phi)*cos(theta) - g],
                 [0.],
                 [0.],
                 [0.],
                 [0.],
                 [0.],
                 [0.]
], dtype = double)
        '''

#        H = array([[1.,   0.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],   # initialize 2-d array
#                   [0.,   1.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  1., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., cos(psi),   sin(psi),  0., -x_estimated[3]*sin(psi)+x_estimated[4]*cos(psi),  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., -sin(psi),   cos(psi),  0., -x_estimated[3]*cos(psi)-x_estimated[4]*sin(psi),  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  1., 0.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 1.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0.,  1., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0.,  0., 1.,   0.,  0., 0.]
#], dtype = double)

#        H = array([[1.,   0.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0., 0.],   # initialize 2-d array
#                   [0.,   1.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0., 0.],
#                   [0.,   0.,  1., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0., 0.],
#                   [0.,   0.,  0., 1.,   0.,  0., 0.,  0., 0.,   0.,  0., 0., 0.],
#                   [0.,   0.,  0., 0.,   1.,  0., 0.,  0., 0.,   0.,  0., 0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  1., 0.,  0., 0.,   0.,  0., 0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 1.,  0., 0.,   0.,  0., 0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0.,  1., 0.,   0.,  0., 0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0.,  0., 1.,   0.,  0., 0., 0.]
#], dtype = double)


#        print time_delay

#        H = array([[1.,   0.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],   # initialize 2-d array
#                   [0.,   1.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  1., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 1.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   1.,  0., 0.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  1., 0.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 1.,  0., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0.,  1., 0.,   0.,  0., 0.],
#                   [0.,   0.,  0., 0.,   0.,  0., 0.,  0., 1.,   0.,  0., 0.]
#], dtype = double)

        #H = eye(12)

#-------------  without xyz dots
        
        H = array([[1.,   0.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],   # initialize 2-d array
                   [0.,   1.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],
                   [0.,   0.,  1., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 1.,  0., 0.,   0.,  0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0.,  1., 0.,   0.,  0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0.,  0., 1.,   0.,  0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0.,  0., 0.,   1.,  0., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  1., 0.],
                   [0.,   0.,  0., 0.,   0.,  0., 0.,  0., 0.,   0.,  0., 1.]
		], dtype = double)
        


#--------- предсказание для матрицы ковариаций-------------------------
        covariances = dot(F,dot(covariances,transpose(F))) + self.Q
#        covariances = dot(F,dot(covariances,transpose(F))) + dot(J,dot(self.Q,transpose(J)))


# ШАГ 2 - КОРРЕКЦИЯ


        PH = dot(covariances,transpose(H))

        HPH = dot(H,PH)

        #print 'HPH', HPH
        S = HPH + self.R

#new
#        S = (S + transpose(S))*0.5
#        #print 'S', S
#        try:
#            L = linalg.cholesky(S).T
#            Linv = linalg.inv(L)
#            U1 = linalg.lstsq(L,eye(L.shape[1]), rcond = 0.01)
#            U2 = linalg.lstsq(L.T.conj(),eye(L.T.conj().shape[1]), rcond = 0.01)
#            W1 = dot(PH, U1[0])
#            K = dot(W1, U2[0])
#        except Exception as e:
#            print "Odnako fignya...\n"
#         #   print x_estimated, covariances, S, PH
#            print e
##            1./0.
#            return x_estimated, covariances


#new

#usual
        K = dot(PH,linalg.inv(S)) #матрица коэфф-а усиления
#        K = 0.*K


        #print 'K:=', K
#    print 'X:=', X

        if PRINT: print ('x', x_estimated)

        x_estimated = x_estimated.reshape(len(x_estimated),1)

#        print 'z pred delta_z', features_d_a, features_prediction_d_a, features_d_a - features_prediction_d_a

#        xx_delta = x_current - x_prediction
#        print x_prediction, dot(H1,x_prediction)
        xx_delta = x_current - dot(H,x_prediction)
#        xx_delta[6] = angle_to_pi(xx_delta[6][0]) #vrode ne vliyaet no ostavim na vsyakii

#usual
        x_delta = dot(K,(xx_delta))
       # print 'x_current - x_prediction x_delta', x_current - x_prediction, x_delta
#        print 'x_old', x_estimated        
        x_estimated = x_estimated + x_delta #коррекция вектора состояния с учетом измеренного выхода

#        x_estimated = x_estimated
        if PRINT: print ('x_new', x_estimated)
#        print 'x_new', x_estimated

        x_estimated = x_estimated.reshape(1,len(x_estimated))[0]

#        self.x_robot_estimated = x_estimated[0:3]


#usual
        covariances = dot((self.I - dot(K,H)),covariances) #обновление матрицы ковариаций

#        print 'covariances', covariances

#new
#        covariances = covariances - dot(W1,transpose(W1))

#        covariances = covariances - dot(U1[0],U1[0].conj().T) #обновление матрицы ковариаций

#        x_features_estimated = delete(x_estimated,range(0,3),0)
#        x_estimated[2] = fetch_angle_to_2pi(x_estimated[2])

        return x_estimated, covariances


