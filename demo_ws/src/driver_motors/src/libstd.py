import math

_2PI = 2. * math.pi

class Desc: pass
E = Desc()

E.RESET = 'R'
E.STOP = 's'
E.MOTOR = 'd'
E.RESET_TICKS = 'p,0,0'
E.READ_TICKS = 'q'
E.NULL = 'a'
E.CAMERA = 'I'

class Params:
    def __init__(self, wheel_radius, distance_between_wheels, devices,
                 k_forward, k_rotation, delta_angle=0):
        self.wheel_radius = wheel_radius
        self.distance_between_wheels = distance_between_wheels
        self.devices = devices
        self.k_forward = k_forward
        self.k_rotation = k_rotation
        self.delta_angle = delta_angle

E.BOT = {}
E.BOT[2107] = Params(0.02076, 0.05365, '/dev/rfcomm2', 0.00005, 0.001)
E.BOT[2150] = Params(0.02069, 0.05290, '/dev/rfcomm1', 0.0000652, 2 * 0.0000652/0.05290)
#E.BOT[2187] = Params(0.02069*0.9, 0.05249, '/dev/rfcomm0', 0.0000652, 2 * 0.0000652/0.05249, delta_angle=0.699842)
E.BOT[2187] = Params(0.02069, 0.05249, '/dev/rfcomm0', 0.0000652*1., 2 * 0.0000652/0.05249*1.02 , delta_angle=0.397829)

E.HUB = Desc()
E.HUB.HOST = 'localhost'
E.HUB.PORT = 50101

E.DATA = Desc()
E.DATA.INIT = 'init'
E.DATA.STATE = 'state'
E.DATA.STOP = 'stop'
E.DATA.U = 'u'

E.NUMBER_OF_BEACON = 4

def limit(value, limit):
    if value > limit:
        value = limit
    elif value < -limit:
        value = -limit
    return value

def fetch_angle_to_2pi(angle):
    while angle > _2PI:
        angle -= _2PI
    while angle < 0:
        angle += _2PI
    return angle

def get_delta_angle(angle_1, angle_2):
    delta_angle = abs(angle_1 - angle_2)
    while delta_angle > _2PI:
        delta_angle -= _2PI
    return delta_angle

def angle_to_pi(angle_rad):
    angle_rad = angle_rad % _2PI
    if angle_rad > math.pi:
        angle_rad -= _2PI
#    while (angle_rad > math.pi):
#        angle_rad -= _2PI
#    while (angle_rad < -math.pi):
#        angle_rad += _2PI
    return angle_rad

def angle_to_2pi(angle_rad):
    while (angle_rad > _2PI):
        angle_rad -= _2PI
    while (angle_rad < -_2PI):
        angle_rad += _2PI
    return angle_rad

def get_phi_ref(delta_x, delta_y):
    if delta_y != 0:
        phi_0 = math.atan(delta_x/delta_y)
    if delta_x < 0 and delta_y < 0:
        phi_ref = phi_0
    elif delta_x < 0 and delta_y > 0 or delta_x >= 0 and delta_y > 0:
        phi_ref = phi_0 + math.pi
    elif delta_x > 0 and delta_y < 0:
        phi_ref = phi_0 + 2. * math.pi
    elif delta_x > 0 and delta_y == 0:
        phi_ref = 3./2. * math.pi
    elif delta_x < 0 and delta_y == 0:
        phi_ref = math.pi / 2.
    elif delta_x == 0:
        phi_ref = 0.
    return phi_ref

def get_delta_phi(delta_phi):
    while delta_phi > math.pi:
        delta_phi -= _2PI
    while delta_phi < -math.pi:
        delta_phi += _2PI
    return delta_phi
