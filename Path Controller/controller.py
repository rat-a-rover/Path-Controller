#!/usr/bin/env python
​
# Simple tracker based on two control loops compenstating for 
# lateral error (off path error) and yaw error
​
# PI controllers are used to correct for lateral and yaw error
# the projected pose is used as a reference to calculate error
​
# Resulting control law is linear velocity control:
#   x_dot is fixed, y_dot controlled, and v is calculated based 
#   on y_dot and x_dot
# and angular velocity control:
#   directly calculated from orientation error

import math
import numpy as np
import rospy
from rover_modules.utils import py2ros
from std_msgs.msg import Float64MultiArray
from base_tracker_type import BaseTrackerType
from operation_modules.trajectories import arc_apply, vel_to_traj

class SimpleTracker(BaseTrackerType):
    def __init__(self, configuration, publish, testing):
        super(SimpleTracker, self).__init__(
            name='simple',
            configuration=configuration,
            publish=publish
        )
        self.TESTING = testing
        if self.TESTING:
            self.yaw_publisher = rospy.Publisher("/operation/tracker/yaw_control",
                                                 Float64MultiArray, queue_size=10)
            self.y_publisher = rospy.Publisher("/operation/tracker/y_control",
                                               Float64MultiArray, queue_size=10)

    def reset(self):
        if self.TESTING:
            self.update_controller()

        self.gains = self.ctrl_config['gains']
        self.limits = self.ctrl_config['limits']
        self.enable = self.ctrl_config['enable']

        self.O_cum_error = 0
        self.y_cum_error = 0

    def update_controller(self):
        self.enable = rospy.get_param(
            '/operation/behaviors/tracker/strategies/simple/' + self.strategy + '/enable')

        self.gains = rospy.get_param(
            '/operation/behaviors/tracker/strategies/simple/' + self.strategy + '/gains')​
    def limit(self, value, key):
        return value if abs(value) < self.limits[key]\
               else self.limits[key] * (1 if value > 0 else -1)

    def apply_PI_control(self, y_error, O_error, x_dot, angle):
        # PI controller for orientation drift error
        omega = -(self.gains['kpO'] * O_error + \
                self.gains['kiO'] * self.O_cum_error)
        omegafb = self.limit(omega, 'w')
       
        
        # PI controller for lateral slip error
        # when y actual > y projected, ydot is negative
        y_dot = (self.gains['kpy'] * y_error + \
                self.gains['kiy'] * self.y_cum_error) * (-1 if x_dot > 0 else 1)​
        # Calculate corrective heading from lateral controller and invoke limit
        heading = math.atan2(y_dot, abs(x_dot))
        headingfb = self.limit(heading, 'h') + angle - math.pi/2

        # Calculate radius, angle, lin_vel, side feedback parameters
        radiusfb, anglefb, velocity, sidefb = \
            vel_to_traj(x_dot, omegafb, headingfb)
        velocityfb = self.limit(velocity, 'v') 
        return radiusfb, anglefb, velocityfb, sidefb, omegafb, headingfb, y_dot​
    def periodic(self, event, data, commands, value):
        if not data['status']['traj_set']:
            return

        if self.TESTING:
            start = rospy.Time.now()

        t = rospy.get_rostime().to_sec() - data['time']['to']
        xa = data['pose']['relative']['x']
        ya = data['pose']['relative']['y']
        Oa = data['pose']['relative']['O']
        va = data['odometry']['v']
        alpha = data['trajectory']['derived']['heading'] 

        _, _, _, _,\
        xp, yp, Op = data['trajectory']['model'](t, xa, ya, Oa, va)

        crab_rotation = np.array([
            [np.cos(-alpha), -np.sin(-alpha)],
            [np.sin(-alpha), np.cos(-alpha)]
        ])
        proj = np.array([xp, yp]).reshape([-1, 1])
        actual = np.array([xa, ya]).reshape([-1, 1])
        proj_straight = np.matmul(crab_rotation , proj)
        actual_straight = np.matmul(crab_rotation , actual)
        _, yar = actual_straight[0], actual_straight[1]
        _, ypr = proj_straight[0], proj_straight[1]
        print(proj_straight, actual_straight)

        O_error = Oa - Op

        y_error = yar - ypr
        # y_error = ya - yp

        if data['trajectory']['params']['type'] != 'point':
            radiusfb, anglefb, velocityfb, sidefb, omegafb, headingfb, y_dot = \
                self.apply_PI_control(
                    y_error,
                    O_error,
                    data['trajectory']['params']['velocity'],
                    data['trajectory']['params']['angle']
                )

            if not self.TESTING or (self.TESTING and self.enable):
                self.publish_tracker_data(arc_apply,
                                        radiusfb,
                                        anglefb,
                                        velocityfb,
                                        sidefb, data, commands)
        else:
            # simple tracker does not control for point turning since there
            # is little to no side slip
            radiusfb = data['trajectory']['params']['radius']
            anglefb = data['trajectory']['params']['angle']
            velocityfb = 0
            sidefb = data['trajectory']['params']['side']
            omegafb = data['trajectory']['params']['velocity']
            headingfb = 0
            y_dot = 0

        self.O_cum_error += O_error
        self.y_cum_error += y_error

        if self.TESTING:
            print('    CONTROLLER ', 'ENABLED ' if self.enable else 'DISABLED ', self.ctr)
            print('    xp: ', xp, 'yp: ', yp, 'Op: ', Op)
            print('    xa: ', xa, 'ya: ', ya, 'Oa: ', Oa)
            print('    xg: ', data['pose']['rel_goal']['x'], 'yg: ', data['pose']['rel_goal']['y'], 'Og: ', data['pose']['rel_goal']['O'])
            print('    O_error: ', O_error, 'y_error', y_error)
            print('    O_cum_error: ', self.O_cum_error, 'y_cum_error', self.y_cum_error)
            print('    omegafb: ', omegafb, 'headingfb: ', headingfb, 'y_dot: ', y_dot)
            print('    radiusfb: ', radiusfb, 'anglefb: ', anglefb,
                  '    velocityfb: ', velocityfb, 'sidefb: ', sidefb)
            print('    Loop Time: ', (rospy.Time.now() - start).to_sec())
            self.yaw_publisher.publish(Float64MultiArray(data=[Oa, Op]))
            self.y_publisher.publish(Float64MultiArray(data=[ya, yp]))