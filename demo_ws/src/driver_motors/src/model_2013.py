from math import sin, cos, pi, sqrt
#from control_simple_square import *
#from control import *
#from control_real import *
#from kalman_quadro import Kalman
#from libstd import angle_to_pi

from numpy import *


M = 0.39    # kg
#m_real = 0.5

G = 9.81     # m/c^2
MG = M * G
Ixx = 0.02
Iyy = 0.02
Izz = 0.02

delay1 = 0
delay = 0

phi_roll_delay = 0
yaw_delay = 0


#0.5 - 3 s
#0.2 - 0.75-1.0 s
#0.15 - same
#0.1 - 0.5 s

tau_common = 0.17 #0.15
tau_real = 0.17

d_common = 0.35 #0.8
d_real = 0.35

k2_common = 1. #0.8
k2_real = 1.

k_u0 = 1.
k_phi = 1.
k_psi = 1.
k_theta = 1.

class Variable_With_Delay(object):

    def __init__(self, init=0, delay=0):
        self.__query = [init] * (delay + 1)

    def set_value(self, new_value):
        tmp = self.__query[0:-1]
        self.__query = [new_value]
        self.__query.extend(tmp)

    def get_value(self):
        return self.__query[-1]

    def get_index(self, index):
        return self.__query[index]

    def get_last_value(self):
        return self.__query[0]

    value = property(get_value, set_value)
    last = property(get_last_value)



class Model_simple:

    def __init__(self, x=0., y=0., z=0., psi=0., phi=0., theta=0., M = 0.4):
        # init state
        self.x = x
        self.y = y
        self.z = z
        self.psi = psi
        self.phi = phi
        self.theta = theta

        self.mu_x = 0.
        self.mu_y = 0.

        # init speed
        class Desc: pass
        self.dot = Desc()
        self.dot.x = 0.
        self.dot.y = 0.
        self.dot.z = 0.
        self.dot.psi = 0.
        self.dot.phi = 0.
        self.dot.theta = 0.

        # init control
        self.u = (0., 0., 0., 0.)


    def set_control(self, u):
        self.u = u

    def step(self, dt):

        # rotate
#        self.dot.phi += (self.u[1] - (Izz - Iyy) *\
#                 self.dot.theta * self.dot.psi) / Ixx * dt
#        self.dot.theta += (self.u[2] - (Ixx - Izz) *\
#                self.dot.phi * self.dot.psi) / Iyy * dt
#        self.dot.psi += self.u[3] / Izz * dt

#        self.psi += self.dot.psi * dt
#        self.phi += self.dot.phi * dt
#        self.theta += self.dot.theta * dt

        # simple model for angles
        self.phi = self.u[1]
        self.theta = self.u[2]
        self.psi = self.u[3]



        # move
        self.dot.x += ((sin(self.psi) * sin(self.phi) +\
                cos(self.psi) * cos(self.phi) * sin(self.theta)) *\
                self.u[0] / M - self.mu_x * self.dot.x) * dt
        self.x += (self.dot.x) * dt

        self.dot.y += ((-cos(self.psi) * sin(self.phi) +\
                sin(self.psi) * cos(self.phi) * sin(self.theta)) *\
                self.u[0] / M - self.mu_y * self.dot.y) * dt
        self.y += (self.dot.y) * dt

        self.dot.z += (cos(self.phi) * cos(self.theta) * self.u[0] - MG) /\
                M * dt
#        self.dot.z = 0.
        self.z += self.dot.z * dt

class Model:

    def __init__(self, x=0., y=0., z=0., psi=0., phi=0., theta=0., tau_psi = tau_common, tau_phi = tau_common, tau_theta = tau_common, d_psi = d_common, d_phi = d_common, d_theta = d_common, k2_psi = 1., k2_phi = k2_common, k2_theta = k2_common, weigth = M, mu = 0.):
#    def __init__(self, x=0., y=0., z=0., psi=0., phi=0., theta=0., tau_psi = 0.1, tau_phi = 0.1, tau_theta = 0.1, M = 0.4):

        # init state
        self.x = x
        self.y = y
        self.z = z

        self.vx = 0.
        self.vy = 0.
        self.vz = 0.

        self.psi = psi
        self.phi = phi
        self.theta = theta
        self.psi_prev = psi
        self.phi_prev = phi
        self.theta_prev = theta
        self.psi_ref = 0.
        self.psi_ref_summ = 0.
        self.psi_ref_prev = 0.


        self.F0x = 0.#-0.15
        self.F0y = 0.# 0.35

#        self.altitude = 600.
#        self.tau_psi = 0.14
#        self.tau_phi = 0.14
#        self.tau_theta = 0.14
        self.tau_psi = tau_psi
        self.tau_phi = tau_phi
        self.tau_theta = tau_theta

        self.d_psi = d_psi #0.35
        self.d_phi = d_phi
        self.d_theta = d_theta


        self.k2_psi = 1./(self.tau_psi**2.)
        self.k2_phi = 1./(self.tau_phi**2.)
        self.k2_theta = 1./(self.tau_theta**2.)

        self.k_psi = k2_psi * self.k2_psi
        self.k_phi = k2_phi * self.k2_phi
        self.k_theta = k2_theta * self.k2_theta

        self.k1_psi = 2.*self.d_psi*self.tau_psi*self.k2_psi
        self.k1_phi = 2.*self.d_phi*self.tau_phi*self.k2_phi
        self.k1_theta = 2.*self.d_theta*self.tau_theta*self.k2_theta

        self.mu_x = mu
        self.mu_y = mu


        self.mass = weigth

        # init speed
        class Desc: pass
        self.dot = Desc()
        self.dot.x = 0.
        self.dot.y = 0.
        self.dot.z = 0.
        self.dot.dx = 0.
        self.dot.dy = 0.
        self.dot.psi = 0.
        self.dot.phi = 0.
        self.dot.theta = 0.

        # init control
        self.u = (0., 0., 0., 0.)


    def set_control(self, u):
        self.u = u

    def step(self, dt):
        '''
        # My model

        self.psi += self.dot.psi * dt
        self.phi += self.dot.phi * dt
        self.theta += self.dot.theta * dt

        self.dot.phi += (self.u[1] - (Izz - Iyy) * self.dot.theta * self.dot.psi) / Ixx * dt
        self.dot.theta += (self.u[2] - (Ixx - Izz) * self.dot.phi * self.dot.psi) / Iyy * dt
        #print(f'u3={self.u[2]} dot_theta={self.dot.theta}')
        self.dot.psi += self.u[3] / Izz * dt
        '''
       #Sasha's model

        self.psi_ref = self.u[3]

        self.dot.phi += (-self.k1_phi * self.dot.phi - self.k2_phi * self.phi + self.k_phi * self.u[1]) * dt
        self.dot.theta += (-self.k1_theta * self.dot.theta - self.k2_theta * self.theta + self.k_theta * self.u[2]) * dt
        self.dot.psi += (-self.k1_psi * self.dot.psi - self.k2_psi * self.psi + self.k_psi * self.psi_ref) * dt
#        self.dot.psi += (-self.k1_psi * self.dot.psi - self.k2_psi * angle_to_pi(self.psi - self.psi_ref)) * dt # hotim ne nakaplivat ugol psi, nado chtobi proisvodnaya ne skakala

        self.psi_prev = self.psi
        self.phi_prev = self.phi
        self.theta_prev = self.theta

        self.psi += self.dot.psi * dt
        self.phi += self.dot.phi * dt
        self.theta += self.dot.theta * dt
        
        
        #print(f'MODEL.py dot_psi={self.dot.psi} dot_phi={self.dot.phi} dot_teta={self.dot.theta}')
        #print(f'psi={self.psi} phi={self.phi} teta={self.theta}')

        # move
        self.dot.x += ((sin(self.psi) * sin(self.phi) + cos(self.psi) * cos(self.phi) * sin(self.theta)) * self.u[0] / self.mass) * dt
        self.x += (self.dot.x) * dt

        self.dot.y += ((-cos(self.psi) * sin(self.phi) + sin(self.psi) * cos(self.phi) * sin(self.theta)) * self.u[0] / self.mass) * dt
        self.y += (self.dot.y) * dt

        R = 0.
        if (self.z<0.3):
            R = self.mass * G

        self.dot.z += (cos(self.phi) * cos(self.theta) * self.u[0] - self.mass * G + R) / self.mass * dt
        self.z += self.dot.z * dt # self.dot.z = 0.
