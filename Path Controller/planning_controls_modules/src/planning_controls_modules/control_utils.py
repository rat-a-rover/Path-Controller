import math
import numpy as np
import numpy.linalg as la
import tf
import time

__NULL_RADIUS__ = 0.0
__INFINITE_RADIUS__ = 100
__NULL_ANGLE__ = math.pi/2

def normalize(angle):
    angle = math.fmod((math.fmod(angle, math.pi*2.0) + 2*math.pi), math.pi*2.0)
    angle = angle - (math.pi*2.0 if angle > math.pi else 0.0)
    return angle

def normalize_twopi(ang):
   return ang % (2*math.pi)

def normalize_mpipi(ang):
    tol = 1e-2
    norm_ang =  ang % (2*math.pi)
    norm_ang = (norm_ang + 2*math.pi) % (2*math.pi)
    if (norm_ang > math.pi + tol):
        norm_ang -= (2*math.pi) 
    return norm_ang

def calc_ang_diff(ang_1, ang_2):
    return math.fabs(math.atan2(math.sin(ang_1 - ang_2),
                                math.cos(ang_1 - ang_2)))

def dist(A, B):
    return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

def sign(A, B):
    return (math.cos(A[2]) * (B[0] - A[0]) + \
            math.sin(A[2]) * (B[1] - A[1])    ) >= 0.0

def get_wheel_config(tf_time, base_name, wheels, radii):
        listener = tf.TransformListener()
        wheel_config = {}
        max_tries = 10
        for wheel_id in wheels:
            trans = []
            for try_num in range(max_tries):
                try:
                    trans, rot = listener.lookupTransform('/' + base_name, 
                                                          '/' + wheel_id, tf_time)
                    break
                except (tf.LookupException, 
                        tf.ConnectivityException,
                        tf.ExtrapolationException):
                    time.sleep(1)
                    if try_num == (max_tries - 1):
                        return {}
            wheel_config[wheel_id] = {
                'x' : round(trans[0], 4),
                'y' : round(trans[1], 4),
                'r' : round(radii[wheel_id], 4)
            }
        return wheel_config

#TODO REMOVE calc_z_rotation, calc_transform, calc_inverse, AND 
#     REPLACE WITH TF LIBRARY EQUIVALENTS
def calc_z_rotation(theta):
    """
    get rotation matrix about z-axis from theta
    """
    R = np.matrix([[math.cos(theta), -math.sin(theta), 0],
                   [math.sin(theta),  math.cos(theta), 0],
                   [           0,             0, 1]])
    return R

def calc_transform(R, T, P):
    """
    rotate P by rotation matrix R, then translate P by T
    can be used to transform P from relative frame to absolute frame
    """
    P_m = np.matrix([P[0], P[1], 0, 1]).T
    T_m = np.matrix([[R[0,0], R[0,1], R[0,2], T[0]],
                     [R[1,0], R[1,1], R[1,2], T[1]],
                     [R[2,0], R[2,1], R[2,2],   0 ],
                     [   0  ,    0  ,   0,      1 ]])
    ret = T_m *  P_m
    return np.array([ret.item(0), ret.item(1)])

def calc_inverse_transform(R, T, P):
    P_m = np.matrix([P[0], P[1], 0, 1]).T
    T_m = np.matrix([[R[0,0], R[0,1], R[0,2], T[0]],
                     [R[1,0], R[1,1], R[1,2], T[1]],
                     [R[2,0], R[2,1], R[2,2],   0 ],
                     [   0  ,    0  ,   0,      1 ]])
    ret = la.inv(T_m) *  P_m
    #print 'P_m' + str(P_m)
    #print 'T_m' + str(T_m)
    #print 'ret' + str(ret)
    return np.array([ret.item(0), ret.item(1)])

