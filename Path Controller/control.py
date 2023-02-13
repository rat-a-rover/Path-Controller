import math
import numpy as np
import rclpy
# from rover_modules.utils import py2ros
from std_msgs.msg import Float64MultiArray
# from base_tracker_type import BaseTrackerType
# from operation_modules.trajectories import arc_apply, vel_to_traj
from rclpy.clock import Clock
from rclpy.node import Node
from rover_msgs.msg import Command
from rover_msgs.msg import Experiment
from nav_msgs.msg import Odometry
from path_control.trajectory_models import *


__NULL_RADIUS__ = 0.0
__INFINITE_RADIUS__ = 100
__NULL_ANGLE__ = math.pi/2

class SimpleTracker(Node):
    def __init__(self): #(self, configuration, publish, testing):
        # super(SimpleTracker, self)._init_(
        #     name='simple',
        #     configuration=configuration,
        #     publish=publish
        # )
        # self.TESTING = testing
        super().__init__('SimpleTracker')
        self.TESTING = False
        self.a_posx,self.a_posy,self.a_posz = 0,0,0
        self.rel_x = 0
        self.rel_y = 0
        self.rel_z = 0
        self.rel_ori   = 0 #verify this based on how orientation(yaw) is given (transform)
        self.vel = 0

        self.gains = {}
        self.O_cum_error = 0
        self.y_cum_error = 0
        self.node = rclpy.create_node('simple_tracker_node')
        # if self.TESTING:
        self.y_publisher = self.node.create_publisher(Command, '/planner/command', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.periodic)        
        self.exp_subscription = self.create_subscription(Command,'/planner/command',self.experiment_callback,10) #both topics are same, how to handle?
        self.localization_subscription = self.create_subscription(Odometry,'/odom',self.localization_callback,10)

    
    def model_creation(self,msg1):

        return {}

    def vel_to_traj(self,linear_velocity, angular_velocity, heading):
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

    def experiment_callback(self,msg1):
        self.TESTING = True
        self.time = self.get_clock().now()
        self.o_posx = self.a_posx
        self.o_posy = self.a_posy
        self.o_posz  = self.a_posz
        self.o_ori  =  0.0 #orientation  #verify orientation
        self.mparams = {}
        self.mtime = 0
        if msg1.type == "turn" :
            self.mparams['type'] = "turn" 
            for name, value in dict(zip(msg1.param_names, msg1.param_values)).items():
                    if name != "side" :
                        self.mparams[name] = float(value)
                    else:
                        self.mparams[name] = (value)
        elif msg1.type == "spot":
            self.mparams['type'] = "spot" 
            for name, value in dict(zip(msg1.param_names, msg1.param_values)).items():
                    if name != "side" :
                        self.mparams[name] = float(value)
                    else:
                        self.mparams[name] = (value)
            self.mparams['radius'] = 0.0
            self.mparams['side'] = 'center'
            self.mparams['heading'] = 0.0
        elif msg1.type == "straight":
            self.mparams['type'] = "straight" 
            for name, value in dict(zip(msg1.param_names, msg1.param_values)).items():
                    if name != "side" :
                        self.mparams[name] = float(value)
                    else:
                        self.mparams[name] = (value)
            self.mparams['radius'] = 1000000.0
            self.mparams['heading'] = 0.0
            self.mparams['side'] = 'left'
        elif msg1.type == "crab":
            self.mparams['type'] = "crab" 
            for name, value in dict(zip(msg1.param_names, msg1.param_values)).items():
                    if name != "side" :
                        self.mparams[name] = float(value)
                    else:
                        self.mparams[name] = (value)
            self.mparams['radius'] = 1000000.0
            self.mparams['side'] = 'left'   #check if side is fixed   
        self.get_logger().info("model created")  
        print(self.mparams)
        self.models = model(self.mtime ,self.mparams)
        
        self.get_logger().info("model created") 
        #create the model

    def localization_callback(self,msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (self.a_posx,self.a_posy,self.a_posz) = (position.x, position.y, position.z)
        # (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        #how will the orientation be given?
        yaw = orientation.z


        self.rel_x = self.o_posx- self.a_posx
        self.rel_y = self.o_posy- self.a_posy
        self.rel_z = self.o_posz- self.a_posz
        self.rel_ori   = self.o_ori - orientation #verify this based on how orientation(yaw) is given (transform)


        self.vel = msg.twist.twist.linear.x

    

    
    def reset(self):
        if self.TESTING:
            self.update_controller()

        # self.gains = self.ctrl_config['gains']
        # self.limits = self.ctrl_config['limits']
        # self.enable = self.ctrl_config['enable']

        self.O_cum_error = 0
        self.y_cum_error = 0

    def update_controller(self):
        # update_controller function would need to be updated with appropriate functionality using ROS2 API

        # Write a parameter code
        # self.enable = self.get_parameter(
        #     '/operation/behaviors/tracker/strategies/simple/' + self.strategy + '/enable').value
        
        # self.gains = self.get_parameter(
        return 

    def limit(self, value, key):
        # return value if abs(value) < self.limits[key]\
        #        else self.limits[key] * (1 if value > 0 else -1)

        # Get the limits
        return value

    def apply_PI_control(self, y_error, O_error, x_dot, angle):
        # PI controller for orientation drift error

        # Find the limits
        self.gains['kpO'] = 1
        self.gains['kiO'] = 1
        self.gains['kpy'] = 1
        self.gains['kiy'] = 1 
        omega = -(self.gains['kpO'] * O_error + self.gains['kiO'] * self.O_cum_error)
        omegafb = self.limit(omega, 'w')
       
        
        # PI controller for lateral slip error
        # when y actual > y projected, ydot is negative
        y_dot = (self.gains['kpy'] * y_error + self.gains['kiy'] * self.y_cum_error) * (-1 if x_dot > 0 else 1)
        # Calculate corrective heading from lateral controller and invoke limit
        heading = math.atan2(y_dot, abs(x_dot))
        headingfb = self.limit(heading, 'h') + angle - math.pi/2

        # Calculate radius, angle, lin_vel, side feedback parameters
        radiusfb, anglefb, velocity, sidefb = self.vel_to_traj(x_dot, omegafb, headingfb)
        velocityfb = self.limit(velocity, 'v') 
        return radiusfb, anglefb, velocityfb, sidefb, omegafb, headingfb, y_dot
        
    def periodic(self):
        # if not data['status']['traj_set']:
        #     return

        if self.TESTING:
            self.start = self.get_clock().now()# check if you need in seconds or nanoseconds

        else :
            return
        
        #t = rospy.get_rostime().to_sec() - data['time']['to'] # Change to ROS2 API
        t = self.start- self.time    ####(calculate the time elapsed)

        # xa = data['pose']['relative']['x'] 
        # ya = data['pose']['relative']['y']
        # Oa = data['pose']['relative']['O']
        # va = data['odometry']['v']
        # alpha = data['trajectory']['derived']['heading'] 


        xa = self.rel_x
        ya = self.rel_y
        Oa = self.rel_ori
        va = self.vel
        alpha = self.mparams['heading'] #where will we get this (heading v/s orientation)
        # _, _, _, _,\
        xp, yp, Op = model(self.mtime,self.mparams)(t, xa, ya, Oa, va)
        
        # self.models.

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

        if self.mparams['type'] != 'point':
            radiusfb, anglefb, velocityfb, sidefb, omegafb, headingfb, y_dot = \
                self.apply_PI_control(
                    y_error,
                    O_error,
                    (self.mparams['velocity']-self.vel)*np.cos(alpha),
                    self.mparams['heading']
                )   #velocity and angle ?

            # if not self.TESTING or (self.TESTING and self.enable):
            #     self.publish_tracker_data(arc_apply,
            #                             radiusfb,
            #                             anglefb,
            #                             velocityfb,
            #                             sidefb, data, commands)
        else:
            # simple tracker does not control for point turning since there
            # is little to no side slip
            radiusfb = self.mparams['radius']
            anglefb = self.mparams['heading']
            velocityfb = 0
            sidefb = self.mparams['side']
            omegafb = self.mparams['velocity']
            headingfb = 0
            y_dot = 0

        self.O_cum_error += O_error
        self.y_cum_error += y_error

        if self.TESTING:
            # print('    CONTROLLER ', 'ENABLED ' if self.enable else 'DISABLED ', self.ctr)
            # print('    xp: ', xp, 'yp: ', yp, 'Op: ', Op)
            # print('    xa: ', xa, 'ya: ', ya, 'Oa: ', Oa)
            # print('    xg: ', data['pose']['rel_goal']['x'], 'yg: ', data['pose']['rel_goal']['y'], 'Og: ', data['pose']['rel_goal']['O'])
            # print('    O_error: ', O_error, 'y_error', y_error)
            # print('    O_cum_error: ', self.O_cum_error, 'y_cum_error', self.y_cum_error)
            # print('    omegafb: ', omegafb, 'headingfb: ', headingfb, 'y_dot: ', y_dot)
            # print('    radiusfb: ', radiusfb, 'anglefb: ', anglefb,
            #       '    velocityfb: ', velocityfb, 'sidefb: ', sidefb)
            # print('    Loop Time: ', (Clock().now().seconds - start).to_sec())
            msg = Command()
            msg.id = 0
            msg.start = float(0.0)
            msg.timeout = float(5) #what will be the timeout
            msg.event = 'START' 
            msg.targets =["mobility_base"]  #change this
            msg.type = str(self.mparams['type'])
            msg.param_names = ["distance","velocity","side","heading"]
            msg.param_values = [str(radiusfb),str(velocityfb),str(sidefb),str(headingfb)]
            self.y_publisher.publish(msg)

def main():
    rclpy.init()
    node = SimpleTracker() # do I need to pass BaseTrackerType?
    rclpy.spin(node)


if __name__ == '__main__':
    main()


