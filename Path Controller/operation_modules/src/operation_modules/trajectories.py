#!/usr/bin/env python

# Samuel Chandler
# samxchandler@protoinnovations.com
    
# Kinematics conversion from 
#     radius of the center of rotation 
#         in meters, 
#     angle from rover x axis to the center of rotation (points on y axis are pi/2 radians)
#         in radians, 
#     velocity (linear) of the rover 
#         in meters / second, 
#     side which side the rover the center of rotation is on
#         left or right
#     wheel locations, x/y position of the wheels from the rover base link 
#         in meters 
#     wheel radii 
#         in meters
# to 
#     wheel steering positions 
#         in radians
#     velocities 
#         in radians / second
    

import math
import numpy as np
import numpy.linalg as la
from planning_controls_modules.control_utils import normalize, __NULL_RADIUS__,\
                                                    __INFINITE_RADIUS__, __NULL_ANGLE__

def arc_apply(radius, angle, velocity, side, icoords):
    '''
    Kinematic conversion for an arc command (fully parameterized)
    '''
    if radius >= __INFINITE_RADIUS__:
        return crab_apply(radius, angle, velocity, side, icoords)

    ocoords = dict()

    #print(radius, angle, velocity, side, icoords)

    xc = radius * math.cos(angle * (1.0 if side == 'left' else -1.0))
    yc = radius * math.sin(angle * (1.0 if side == 'left' else -1.0))
    # print('xc', xc, 'yc', yc)

    for key, icoord in icoords.iteritems():
        # print(key)
        x = xc - icoord['x']
        y = yc - icoord['y']
        r = icoord['r']
        # print('x', x, 'y', y)
        O = math.atan2(
                y, # * (1.0 if side == 'left' else -1.0),
                x 
            ) - math.pi/2 * (1.0 if side == 'left' else -1.0)
        # ) * (1.0 if side == 'left' else -1.0)
        v = velocity / r * (math.sqrt(x**2 + y**2) / radius)
        print('O', O, 'v', v)

        ocoords[key] = {
            'O' : O,
            'v' : v,
        }

    return ocoords

def straight_apply(radius, angle, velocity, side, icoords):
    '''
    Kinematic conversion for an straight command 
    (constraints: radius is inf and angle is 0)
    '''
    ocoords = dict()

    for key, icoord in icoords.iteritems():
        ocoords[key] = {
            'O' : 0.0,
            'v' : velocity / icoord['r']
            #'v' : velocity
        }

    return ocoords

def point_apply(radius, angle, velocity, side, icoords):
    '''
    Kinematic conversion for an point command 
    (constraints: radius is 0)

    HERE VELOCITY INPUT IS ANGULAR VELOCITY OF ROVER in RAD/S
    '''
    ocoords = dict()

    xc = 0.0
    yc = 0.0

    for key, icoord in icoords.iteritems():
        #print(key)
        x = xc - icoord['x']
        y = yc - icoord['y']
        r = icoord['r']
        #print('x', x, 'y', y)
        O = normalize(
            math.atan2(y, x) - \
            math.pi/2 - \
            (math.pi if key.startswith('L') else 0.0)
        )
        # convert velocity from rad/s at rover level to wheel rad/s
        # rover radius formed by point turn kinematics
        rover_radius = (x ** 2 + y ** 2) ** 0.5
        print('x ', x, 'y ', y, 'rover_radius ', rover_radius)
        # wheel_lin vel is simply rover radius time commanded vehicle
        # rotation speed in rad/s
        wheel_lin_vel = velocity * rover_radius
        print('wheel_lin_vel ', wheel_lin_vel)
        # convert wheel linear velocity to wheel rad/s using wheel radius
        v = wheel_lin_vel / r  *\
            (-1.0 if key.startswith('L') else 1.0)


        ocoords[key] = {
            'O' : O,
            'v' : v,
        }

    return ocoords

def crab_apply(radius, angle, velocity, side, icoords):
    '''
    Kinematic conversion for an crab command 
    (constraints: radius is inf)
    '''
    if angle == __NULL_ANGLE__:
        return straight_apply(radius, angle, velocity, side, icoords)

    ocoords = {} 
    for key, icoord in icoords.iteritems():
        ocoords[key] = {
            'O' : (angle - __NULL_ANGLE__)\
                  * (1.0 if side  == 'left' else -1.0),
            'v' : velocity / icoord['r']
        }

    return ocoords

def traj_to_vel(radius, angle, traj_velocity, side):
    '''
    Derive linear_velocity, angular velocity and heading from
    trajectory command parameters (radius, angle, traj_velocity, side)
    '''
    radius_thresh = .01
    # check if normal trajectory of point turn
    if radius > radius_thresh:
        angular_velocity = traj_velocity/radius * (-1 if (side == 'right') else 1)
        heading = ((angle - math.pi/2) if (angle > 0) 
                  else (angle + math.pi/2)) * (-1 if side == 'right' else 1)
        linear_velocity = traj_velocity
    else:
        heading = 0
        angular_velocity = traj_velocity
        linear_velocity = 0 

    return linear_velocity, angular_velocity, heading

def vel_to_traj(linear_velocity, angular_velocity, heading):
    '''
    Derive radius, angle, linear_velocity, and side from 
    linear_velocity, angular_velocity and heading
    '''
    radius_thresh = 0.01
    angular_velocity_thresh = 0.001

    if math.fabs(angular_velocity) > angular_velocity_thresh:
        radius = math.fabs(linear_velocity / angular_velocity)
    else:
        radius = __INFINITE_RADIUS__

    # check if normal trajectory of point turn
    if radius > radius_thresh:
        side   = ('left' if ((angular_velocity > 0) and (linear_velocity > 0))
                        or ((angular_velocity < 0) and (linear_velocity < 0))
                 else 'right')
        angle  = (heading + math.pi/2) if (side == 'left') \
                 else (heading - math.pi/2) * -1
        traj_velocity = linear_velocity
    else:
        traj_velocity = angular_velocity
        side = 'left'
        angle = __NULL_ANGLE__

    return radius, angle, traj_velocity, side

