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
from geometry_msgs.msg import PoseStamped
from path_control.trajectory_models import *
from tf_transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, quaternion_from_matrix, euler_from_matrix
# from pylot.utils import Location, Pose, Rotation, Transform

__NULL_RADIUS__ = 0.0
__INFINITE_RADIUS__ = 1000
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
        self.a_posx,self.a_posy,self.a_posz,self.roll,self.pitch,self.yaw = 0.0,0.0,0.0,0.0,0.0,0.0
        self.rel_x = 0
        self.rel_y = 0
        self.rel_z = 0
        self.o_posx,self.o_posy,self.o_posz,self.o_ori = self.a_posx,self.a_posy,self.a_posz ,3.14
        self.rel_ori   = 0 #verify this based on how orientation(yaw) is given (transform)
        self.vel = 0
        self.previous_time = 0
        self.gains = {}
        self.O_cum_error = 0
        self.y_cum_error = 0
        self.ang_vel = 0.0
        self.event = "STOP"
        self.node = rclpy.create_node('simple_tracker_node')
        self.LOC_READY = False
        # if self.TESTING:
        self.y_publisher = self.node.create_publisher(Command, '/planner/command', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.periodic)        
        self.exp_subscription = self.create_subscription(Experiment,'/experiment',self.experiment_callback,10) #both topics are same, how to handle?
        self.localization_subscription = self.create_subscription(Odometry,'/localization_data' ,self.localization_callback,10)
        self.pose_subscription = self.create_subscription(PoseStamped, '/ros2_aruco/pose_filtered', self.pose_callback, 10)
        self.base_link_wrt_cam = np.array([ 0, -1, 0, 0,
                                           0, 0, -1, 0,
                                           1, 0, 0, 0,
                                           0, 0, 0, 1    ]).reshape(4, 4)
    
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
            # angle  = (heading + math.pi/2) if (side == 'left') \
            #         else (heading - math.pi/2) * -1
            angle  = (heading ) if (side == 'left') \
                    else (heading) * -1
            traj_velocity = linear_velocity
        else:
            traj_velocity = angular_velocity
            side = 'left'
            angle = __NULL_ANGLE__

        return radius, angle, traj_velocity, side

    def experiment_callback(self,msg1):
        
        self.time = self.get_clock().now()
        
        if  msg1.cmd.event == 'START' :
            self.event = 'START'
            self.get_logger().info("In experiment call back, start")
            self.TESTING = True
        
            self.o_posx = self.a_posx
            self.o_posy = self.a_posy
            self.o_posz = self.a_posz
            self.o_ori  = self.yaw #orientation  #verify orientation

            self.mparams = {}
            self.mtime = 0
            if msg1.cmd.type == "turn" :
                self.mparams['type'] = "turn" 
                for name, value in dict(zip(msg1.cmd.param_names, msg1.cmd.param_values)).items():
                        if name != "side" :
                            self.mparams[name] = float(value)
                        else:
                            self.mparams[name] = (value)
                if self.mparams['side'] == "right":
                    mult = -1
                else:
                    mult = 1
                self.ang_vel= self.mparams['velocity'] / self.mparams['distance'] * mult
            elif msg1.cmd.type == "spot":
                self.mparams['type'] = "spot" 
                for name, value in dict(zip(msg1.cmd.param_names, msg1.cmd.param_values)).items():
                        if name != "side" :
                            self.mparams[name] = float(value)
                        else:
                            self.mparams[name] = (value)
                self.mparams['distance'] = 0.0
                self.mparams['side'] = 'center'
                self.mparams['heading'] = 0.0
            elif msg1.cmd.type == "straight":
                self.mparams['type'] = "straight" 
                for name, value in dict(zip(msg1.cmd.param_names, msg1.cmd.param_values)).items():
                        if name == "yaw_error" :
                            self.o_ori -= float(value)
                        elif name != "side" :
                            self.mparams[name] = float(value)
                        else:
                            self.mparams[name] = (value)
                self.mparams['distance'] = 1000000.0
                self.mparams['heading'] = 0.0
                self.mparams['side'] = 'left'
            elif msg1.cmd.type == "crab":
                self.mparams['type'] = "crab" 
                for name, value in dict(zip(msg1.cmd.param_names, msg1.cmd.param_values)).items():
                        if name != "side" :
                            self.mparams[name] = float(value)
                        else:
                            self.mparams[name] = (value)
                self.mparams['distance'] = 1000000.0
                self.mparams['side'] = 'left'   #check if side is fixed   
            self.get_logger().info("model created")  
            print(self.mparams)
            self.models = model(self.mtime ,self.mparams)


        elif  msg1.cmd.event == 'STOP' :

            self.get_logger().info("In experiment call back, stop")
            self.TESTING = True
            self.event = 'STOP'
            self.o_posx = self.a_posx
            self.o_posy = self.a_posy
            self.o_posz  = self.a_posz
            self.o_ori  =  self.yaw #orientation  #verify orientation
        
            self.mparams = {}
            self.mtime = 0
            self.mparams['type'] = 'straight'
            self.mparams['distance'] = 100000
            self.mparams['heading'] = 0.0
            self.mparams['velocity'] = 0.0
            self.mparams['side'] = 0.0
            print(self.mparams)
            self.models = model(self.mtime ,self.mparams)
            # self.get_logger().info("model created") 
            
    def pose_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        (self.a_posx,self.a_posy,self.a_posz) = (position.x, position.y, position.z)
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(quat)
        # self.yaw = abs(self.yaw)
        # self.yaw = self.yaw + 3.14159
        self.get_logger().debug(f'In localization data: {position.x}, {position.y}, {position.z}, {self.roll}, {self.pitch}, {self.yaw}')

    def localization_callback(self,msg):
        # position = msg.pose.pose.position
        # orientation = msg.pose.pose.orientation
        list2 = [[msg.twist.twist.linear.x ],[msg.twist.twist.linear.y],[msg.twist.twist.linear.z],[1]]
        # (self.a_posx,self.a_posy,self.a_posz) = (position.x, position.y, position.z)
        # quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        # self.roll, self.pitch, self.yaw = euler_from_quaternion(quat)
        # self.yaw = abs(self.yaw)
        # print("yaw =" , self.yaw)
        # print("o_yaw =" , self.o_ori)
        # print("In localization data: ",position.x, position.y, position.z,self.roll, self.pitch, self.yaw )
        self.rotation = euler_matrix(np.degrees(self.pitch), np.degrees(self.yaw),
                            np.degrees(self.roll))
        self.velocity_rover_frame =  np.linalg.inv(self.rotation)@np.array(list2)
        # print("Velocity In lrover frame vector: ",self.velocity_rover_frame )
        self.vel = np.linalg.norm(self.velocity_rover_frame[0:2]) #how to handle sign

        # Initialize start position on reception of first localization message
        if not self.LOC_READY:
            self.LOC_READY = True

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec*(10**-9)  #incase velocity is not given
        self.relative_roll, self.relative_pitch, self.relative_yaw = euler_from_matrix(self.rotation)
        self.rel_x = self.o_posx- self.a_posx
        self.rel_y = self.o_posy- self.a_posy
        self.rel_z = self.o_posz- self.a_posz
        self.rel_ori   = self.o_ori-self.yaw #verify this based on how orientation(yaw) is given (transform)
        self.rel_ori = (self.rel_ori)
        # print("Origin \n",self.o_posx, self.o_posy, self.o_posz, self.o_ori)
        
        # print("Relative Orientation = ",self.rel_ori)
        # print("In localization data Relative Data: ",self.rel_x, self.rel_y, self.rel_z,self.rel_ori)
        # self.vel_x = np.linalg.inv(self.rotation)*msg.twist.twist.linear.x 
        
        # print("Velocity In localization data: ",list2 )
        # print(self.rotation)
        
    
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
        self.gains['kpO'] = 0.01
        self.gains['kiO'] = 0.000
        self.gains['kpy'] = 0.1
        self.gains['kiy'] = 0.003 
        omega = -(self.gains['kpO'] * O_error + self.gains['kiO'] * self.O_cum_error)
        # print("omega ==",omega)
        omegafb = self.limit(omega, 'w')
       
        
        # PI controller for lateral slip error
        # when y actual > y projected, ydot is negative
        y_dot = (self.gains['kpy'] * y_error + self.gains['kiy'] * self.y_cum_error) * (-1 if x_dot > 0 else 1)
        if y_dot < 0.01 :
            y_dot = 0.0
        # print("y_dot ==",y_dot)
        # print("x_dot ==",x_dot)
        # Calculate corrective heading from lateral controller and invoke limit
        heading = math.atan2(y_dot, abs(x_dot))
        headingfb = self.limit(heading, 'h') 
        # print("heading ==",headingfb)
        # Calculate radius, angle, lin_vel, side feedback parameters
        radiusfb, anglefb, velocity, sidefb = self.vel_to_traj(x_dot, self.ang_vel+omegafb, headingfb)
        velocityfb = self.limit(velocity, 'v') 
        return radiusfb, anglefb, velocityfb, sidefb, omegafb, headingfb, y_dot
        
    def periodic(self):
        

        if self.TESTING and self.LOC_READY:
            self.start = self.get_clock().now()# check if you need in seconds or nanoseconds

        else :
            return
        
        
        t = self.start- self.time    ####(calculate the time elapsed)




        xa = self.rel_x
        ya = self.rel_y
        Oa = self.rel_ori
        # va = self.mparams['velocity']
        va = self.vel
        alpha = self.mparams['heading'] #where will we get this (heading v/s orientation)
        # _, _, _, _,\
        xp, yp, Op = model(self.mtime,self.mparams)(t, xa, ya, Oa, va)
        
        

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
        # print('projected points',proj_straight, actual_straight)
        # print('projected orientation Oa , Op',Oa, Op)
        O_error = Oa - Op

        y_error = yar - ypr
        # y_error = ya - yp

        if self.mparams['type'] != 'spot':
            radiusfb, anglefb, velocityfb, sidefb, omegafb, headingfb, y_dot = \
                self.apply_PI_control(
                    y_error,
                    O_error,
                    (self.mparams['velocity']),
                    self.mparams['heading']
                )   
            cmd_type = 'turn'
        else:
            # simple tracker does not control for point turning since there
            # is little to no side slip
            radiusfb = self.mparams['distance']
            anglefb = self.mparams['heading']
            velocityfb = self.mparams['velocity']
            sidefb = self.mparams['side']
            omegafb = self.mparams['velocity']
            headingfb = self.mparams['heading']
            cmd_type = 'spot'
            y_dot = 0

        self.O_cum_error += O_error
        self.y_cum_error += y_error

        if self.TESTING:
            # log = self.get_logger()
            # log.info('    CONTROLLER ', 'ENABLED ' if self.enable else 'DISABLED ', self.ctr)
            # log.info('    xp: ', xp, 'yp: ', yp, 'Op: ', Op)
            # log.info('    xa: ', xa, 'ya: ', ya, 'Oa: ', Oa)
            # # print('    xg: ', data['pose']['rel_goal']['x'], 'yg: ', data['pose']['rel_goal']['y'], 'Og: ', data['pose']['rel_goal']['O'])
            # log.info('    O_error: ', O_error, 'y_error', y_error)
            # log.info('    O_cum_error: ', self.O_cum_error, 'y_cum_error', self.y_cum_error)
            # log.info('    omegafb: ', omegafb, 'headingfb: ', headingfb, 'y_dot: ', y_dot)
            # log.info('    radiusfb: ', radiusfb, 'anglefb: ', anglefb,
            #       '    velocityfb: ', velocityfb, 'sidefb: ', sidefb)
            # # log.info('    Loop Time: ', (Clock().now().seconds - start).to_sec())
            msg = Command()
            msg.id = 0
            msg.start = float(0.0)
            msg.timeout = float(5) #what will be the timeout
            msg.event = self.event 
            msg.targets =["mobility_base"]  #change this
            msg.type =cmd_type
            msg.param_names = ["distance","velocity","side","heading"]
            msg.param_values = [str(radiusfb),str(velocityfb),str(sidefb),str(anglefb)]
            self.y_publisher.publish(msg)

def main():
    rclpy.init()
    node = SimpleTracker() # do I need to pass BaseTrackerType?
    rclpy.spin(node)


if __name__ == '__main__':
    main()


