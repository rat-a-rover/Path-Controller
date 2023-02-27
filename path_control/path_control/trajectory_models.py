import math
import numpy as np
import numpy.linalg as la
# from planning_controls_modules.control_utils import normalize_mpipi, dist,\
#                                                     __NULL_RADIUS__,\
#                                                     __INFINITE_RADIUS__,\
#                                                     __NULL_ANGLE__



__NULL_RADIUS__ = 0.0
__INFINITE_RADIUS__ = 100
__NULL_ANGLE__ = math.pi/2


def normalize_mpipi(ang):
    tol = 1e-2
    norm_ang =  ang % (2*math.pi)
    norm_ang = (norm_ang + 2*math.pi) % (2*math.pi)
    if (norm_ang > math.pi + tol):
        norm_ang -= (2*math.pi) 
    return norm_ang

def dist(A, B):
    return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

def arc_locate(radius, angle, velocity, side, icoords):
    ocoords = dict()

    for key, icoord in icoords.items():
        ocoords[key] = {
            'x' : (- radius * math.sin((angle - __NULL_ANGLE__))            + \
                     radius * math.sin((angle - __NULL_ANGLE__)             + \
                                       (velocity / radius) * icoord['t'])),
            'y' : (  radius * math.cos((angle - __NULL_ANGLE__))            + \
                   - radius * math.cos((angle - __NULL_ANGLE__)             + \
                                       (velocity / radius) * icoord['t']))  * \
                  (1.0 if side == 'left' else -1.0),
            'O' : normalize_mpipi(
                    ((velocity / radius) * icoord['t'])         * \
                    (1.0 if side == 'left' else -1.0)
                  ),
        }

    return ocoords

def arc_project(radius, angle, velocity, side, icoords):
    #print('arc_project')

    ocoords = dict()

    origin = {
        'x' : radius * math.cos((angle)                             * \
                                (1.0 if side == 'left' else -1.0)),
        'y' : radius * math.sin((angle)                             * \
                                (1.0 if side == 'left' else -1.0)),
    }
    #print('origin')
    #print(origin)

    for key, icoord in icoords.items():
        vector = {
            'x' : icoord['x'] - origin['x'],
            'y' : icoord['y'] - origin['y'],
        }
        #print('vector')
        #print(vector)

        modulus = math.sqrt(vector['x']**2 + vector['y']**2)
        #print('modulus')
        #print(modulus)

        theta = math.acos(
            (- math.cos((angle)                            * \
                        (1.0 if side == 'left' else -1.0)) * \
               vector['x']                                 + \
             - math.sin((angle)                            * \
                        (1.0 if side == 'left' else -1.0)) * \
               vector['y'])                                / \
               modulus
        )
        #print('theta')
        #print(theta)

        idt = {
            'forward'  : { 't' : (              theta) * radius / (velocity + 1e-6)},
            'backward' : { 't' : (2 * math.pi - theta) * radius / (velocity + 1e-6)},
        }
        #print('idt')
        #print(idt)

        vector = {
            'x' : (radius / modulus) * vector['x'] + origin['x'],
            'y' : (radius / modulus) * vector['y'] + origin['y'],
        }
        #print('vector')
        #print(vector)

        odt = arc_locate(radius, angle, velocity, side, idt)
        #print('odt')
        #print(odt)

        if dist((odt['forward']['x'], odt['forward']['y']),     \
                (vector['x'], vector['y'])) <                   \
           dist((odt['backward']['x'], odt['backward']['y']),   \
                (vector['x'], vector['y'])):
            O = odt['forward']['O']
        else:
            O = odt['backward']['O']
        #print('O')
        #print(O)

        ocoords[key] = {
            'x' : vector['x'],
            'y' : vector['y'],
            'O' : normalize_mpipi(O),
        }

    return ocoords

def straight_locate(radius, angle, velocity, side, icoords):
    ocoords = dict()

    for key, icoord in icoords.items():
        ocoords[key] = {
            'x' : velocity * icoord['t'],
            'y' : 0.0,
            'O' : 0.0,
        }

    return ocoords

def straight_project(radius, angle, velocity, side, icoords):
    ocoords = dict()

    for key, icoord in icoords.items():
        ocoords[key] = {
            'x' : icoord['x'],
            'y' : 0.0,
            'O' : 0.0,
        }

    return ocoords

def point_locate(radius, angle, velocity, side, icoords):
    ocoords = dict()
    for key, icoord in icoords.items():
        ocoords[key] = {
            'x' : 0.0,
            'y' : 0.0,
            'O' : normalize_mpipi(
                     # velocity here is angular velocity of rover
                     velocity * icoord['t']         * \
                     (1.0 if side == 'left' else -1.0)
                   )
        }

    return ocoords

def point_project(radius, angle, velocity, side, icoords):
    ocoords = dict()
    for key, icoord in icoords.items():
        ocoords[key] = {
            'x' : 0.0,
            'y' : 0.0,
            'O' : normalize_mpipi(icoord['O'])
        }

    return ocoords

def crab_locate(radius, angle, velocity, side, icoords):
    ocoords = dict()

    for key, icoord in icoords.items():
        ocoords[key] = {
            'x' : velocity                                      * \
                  math.cos((angle - __NULL_ANGLE__)             * \
                           (1.0 if side == 'left' else -1.0))   * \
                  icoord['t'],
            'y' : velocity                                      * \
                  math.sin((angle - __NULL_ANGLE__)             * \
                           (1.0 if side == 'left' else -1.0))   * \
                  icoord['t'],
            'O' : 0.0,
        }

    return ocoords

def crab_project(radius, angle, velocity, side, icoords):
    ocoords = dict()

    for key, icoord in icoords.items():
        modulus = math.cos(angle - __NULL_ANGLE__) * icoord['x']   + \
                  math.sin(angle - __NULL_ANGLE__) * icoord['y']

        ocoords[key] = {
            'x' : modulus                                           * \
                  math.cos((angle - __NULL_ANGLE__)                 * \
                           (1.0 if side == 'left' else -1.0)),
            'y' : modulus                                           * \
                  math.sin((angle - __NULL_ANGLE__)                 * \
                           (1.0 if side == 'left' else -1.0)),
            'O' : 0.0,
        }

    return ocoords     
        

def model(mtime, mparams):
    def model_handler(t, x, y, O, v):
        # ocoords_locate = LOCATE[mparams['type']](
        #     mparams['radius'],
        #     mparams['heading'],
        #     mparams['velocity'],
        #     mparams['side'],
        #     { 'rover' : { 't' : t }}
        # )
        # xd = ocoords_locate['rover']['x']
        # yd = ocoords_locate['rover']['y']
        # Od = ocoords_locate['rover']['O']
        # vd = mparams['velocity']

        ocoords_project = PROJECT[mparams['type']](
            mparams['distance'],
            mparams['heading'],
            mparams['velocity'],
            mparams['side'],
            { 'rover' : { 't' : t, 'x' : x, 'y' : y , 'O': O}}
        )
        xp = ocoords_project['rover']['x']
        yp = ocoords_project['rover']['y']
        Op = ocoords_project['rover']['O']

        return  xp, yp, Op
    # print("Hello ROS2")
    return model_handler

LOCATE = {
    'turn'      :      arc_locate,
    'straight' : straight_locate,
    'crab'     :     crab_locate,
    'point'    :    point_locate,
}

PROJECT = {
    'turn'      :      arc_project,
    'straight' : straight_project,
    'crab'     :     crab_project,
    'spot'    :    point_project,
}
