import math
import numpy as np
import rclpy
from rover_modules.utils import py2ros
from std_msgs.msg import Float64MultiArray
# from base_tracker_type import BaseTrackerType
from operation_modules.trajectories import arc_apply, vel_to_traj
from rclpy.clock import Clock
from rclpy.node import Node
from rover_msgs.msg import Command
from rover_msgs.msg import Experiment
from nav_msgs.msg import Odometry
from trajectory_models import *
class SimpleTracker():
    def __init__(self): #(self, configuration, publish, testing):
        # super(SimpleTracker, self)._init_(
        #     name='simple',
        #     configuration=configuration,
        #     publish=publish
        # )
        # self.TESTING = testing
        super().__init__('SimpleTracker')
        self.node = rclpy.create_node('simple_tracker_node')
        # if self.TESTING:
        self.y_publisher = self.node.create_publisher(Command, '/planner/command', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.periodic)        
        self.exp_subscription = self.create_subscription(Experiment,'/planner/command',self.experiment_callback,10) #both topics are same, how to handle?
        self.localization_subscription = self.create_subscription(Odometry,'/odom',self.localization_callback,10)
   
    
    def model_creation(self,msg1):

        return {}


    def experiment_callback(self,msg1):
        self.TESTING = True
        self.time = Clock().now().seconds
        self.o_posx = self.a_posx
        self.o_posy = self.a_posy
        self.o_poz  = self.a_posz
        self.o_ori  =  orientation  #verify orientation
        self.mparams = {}
        self.mtime = 0
        if msg1.type == "turn" :
            for name, value in dict(zip(msg1.param_names, msg1.param_values)).items():
                    self.mparams[name] = float(value)
        elif msg1.type == "spot":
            for name, value in dict(zip(msg1.param_names, msg1.param_values)).items():
                self.mparams[name] = float(value)
            self.mparams['radius'] = 0.0
            self.mparams['side'] = 'center'
            self.mparams['heading'] = 0.0
        elif msg1.type == "straight":
            for name, value in dict(zip(msg1.param_names, msg1.param_values)).items():
                self.mparams[name] = float(value)
            self.mparams['radius'] = 1000000.0
            self.mparams['heading'] = 0.0
            self.mparams['side'] = 'left'
        elif msg1.type == "crab":
            for name, value in dict(zip(msg1.param_names, msg1.param_values)).items():
                self.mparams[name] = float(value)
            self.mparams['radius'] = 1000000.0
            self.mparams['side'] = 'left'   #check if side is fixed     
        self.models = model(self.mtime ,self.mparams)
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
        
        self.gains = self.get_parameter(
            '/operation/behaviors/tracker/strategies/simple/' + self.strategy + '/gains').value  ###what about parameters (fixed)

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
        return radiusfb, anglefb, velocityfb, sidefb, omegafb, headingfb, y_dot
        
    def periodic(self):
        # if not data['status']['traj_set']:
        #     return

        if self.TESTING:
            start = Clock().now().seconds # check if you need in seconds or nanoseconds

        else :
            return
        
        #t = rospy.get_rostime().to_sec() - data['time']['to'] # Change to ROS2 API
        t = Clock().now().seconds - self.time    ####(calculate the time elapsed)

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
        _, _, _, _,\
        xp, yp, Op = self.models.model_handler((t, xa, ya, Oa, va))
        
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
            msg.start = 0
            msg.timeout = t #what will be the timeout
            msg.event = 'START' 
            msg.targets =''  #change this
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

