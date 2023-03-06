import math
import numpy as np
import rclpy
# from rover_modules.utils import py2ros
from std_msgs.msg import Bool
# from base_tracker_type import BaseTrackerType
# from operation_modules.trajectories import arc_apply, vel_to_traj
from rclpy.clock import Clock
from rclpy.node import Node
from rover_msgs.msg import Command, SimpleStatus
from rover_msgs.msg import Experiment
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from path_control.trajectory_models import *
from tf_transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, quaternion_from_matrix, euler_from_matrix, quaternion_matrix, compose_matrix, decompose_matrix

__NULL_RADIUS__ = 0.0
__INFINITE_RADIUS__ = 1000
__NULL_ANGLE__ = math.pi/2

class SimpleTracker(Node):
    def __init__(self, name):
        super().__init__(name)

        self.ACTIVE = False
        self.LOC_READY = False
        self.a_posx,self.a_posy,self.a_posz,self.a_roll,self.a_pitch,self.a_yaw = 0.0,0.0,0.0,0.0,0.0,0.0
        self.o_posx,self.o_posy,self.o_posz,self.o_roll,self.o_pitch,self.o_yaw = self.a_posx,self.a_posy,self.a_posz ,self.a_roll,self.a_pitch,self.a_yaw
        self.rel_x = 0
        self.rel_y = 0
        self.rel_z = 0
        self.rel_ori = 0
        self.vel = 0
        self.previous_time = 0
        self.gains = {}
        self.O_cum_error = 0
        self.y_cum_error = 0
        self.ang_vel = 0.0
        self.event = "STOP"   
        self.limits = {'h': 0.524, 'w': 0.2, 'v': 0.1}
        self.rel_goal_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Abbrievation key:
        # w: world frame
        # s: start frame
        # o: origin
        # c: current rover frame
        # g: goal frame
        self.T_o_w = np.eye(4, 4)
        self.T_s_w = np.eye(4, 4)
        self.T_c_w = np.eye(4, 4)
        self.T_g_s = np.eye(4, 4)

        self.GOAL_REACHED = False

        self.exp_subscription = self.create_subscription(Experiment,'/experiment',self.experiment_callback, 50)
        self.localization_subscription = self.create_subscription(Odometry,'/localization_data' ,self.localization_callback, 50)
        self.reset_subscriber = self.create_subscription(Bool, '/reset', self.reset_callback, qos_profile=1)
        self.y_publisher = self.create_publisher(Command, '/planner/command', 10)
        self.goal_publisher = self.create_publisher(Bool, '/reached_goal', 10)
        self.debug_pub = self.create_publisher(SimpleStatus, '/tracker/debug', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.periodic) 

    # def vel_to_traj(self,linear_velocity, angular_velocity, heading):
    #     '''
    #     Derive radius, angle, linear_velocity, and side from 
    #     linear_velocity, angular_velocity and heading
    #     '''
    #     radius_thresh = 0.01
    #     angular_velocity_thresh = 0.001

    #     if math.fabs(angular_velocity) > angular_velocity_thresh:
    #         radius = math.fabs(linear_velocity / angular_velocity)
    #     else:
    #         radius = __INFINITE_RADIUS__

    #     # check if normal trajectory of point turn
    #     if radius > radius_thresh:
    #         side   = ('left' if ((angular_velocity > 0) and (linear_velocity > 0))
    #                         or ((angular_velocity < 0) and (linear_velocity < 0))
    #                 else 'right')

    #         angle  = (heading ) if (side == 'left') \
    #                 else (heading) * -1
    #         traj_velocity = linear_velocity
    #     else:
    #         traj_velocity = angular_velocity
    #         side = 'left'
    #         angle = __NULL_ANGLE__

    #     return radius, angle, traj_velocity, side

    def vel_to_traj(self, linear_velocity, angular_velocity, heading):
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
    
    def check_reached_goal(self, mparams, xc, yc, Oc):
        if self.GOAL_REACHED:
            return # don't send goal reached message more than once

        _, _, angles, trans, _ = decompose_matrix(self.T_g_s)
        xg = trans[0]
        yg = trans[1]
        Og = angles[2]
        eps = 0.0
        if mparams['type'] == "straight":
            # delta = np.sqrt((xg - xc)**2 + (yg - yc)**2)
            delta = xg - np.sqrt(xc**2 + yc**2)
            eps = 0.04 # meters
            self.get_logger().info(f"Checking goal reached: delta = {delta}, xc = {xc}, yc = {yc}, xg = {xg}, yg = {yg}")
        elif mparams['type'] == "spot":
            delta = np.abs(Og - Oc)
            eps = 0.05236 # ~3deg
            self.get_logger().info(f"Checking goal reached: delta = {delta}, Oc = {Oc}, Og = {Og}")

        if delta < eps:
            goal_msg = Bool()
            goal_msg.data = True
            self.goal_publisher.publish(goal_msg)
            self.GOAL_REACHED = True
            self.get_logger().info("Sent goal reached message")

    def localization_callback(self,msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        T = compose_matrix(angles=np.array([roll, pitch, yaw]), translate=np.array([position.x, position.y, position.z]))
        self.T_c_w = np.copy(T)

        if not self.LOC_READY:
            # Set the origin w.r.t world frame to the current rover frame
            self.T_o_w = np.copy(self.T_c_w)
            self.T_s_w = np.copy(self.T_o_w)
            self.LOC_READY = True

    def reset_callback(self, msg):
        if msg.data:
            # Set the origin w.r.t world frame to the current rover frame
            self.T_o_w = np.copy(self.T_c_w)
            self.T_s_w = np.copy(self.T_o_w)
            self.T_g_s = np.eye(4, 4)
            self.O_cum_error = 0
            self.y_cum_error = 0

    def experiment_callback(self,msg1):
        ts = self.get_clock().now().to_msg()
        self.start_time = ts.sec + ts.nanosec * 1e-9
        timeout = msg1.cmd.timeout
        
        if  msg1.cmd.event == 'START' :
            self.event = 'START'
            self.get_logger().info("In experiment call back, start")
            self.ACTIVE = True

            # Calculate new start transform based previous start frame and relative goal frame
            self.T_s_w = self.T_s_w @ self.T_g_s

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

                # Create T_g_s for point turn
                Od = self.mparams['velocity'] * timeout
                self.T_g_s = compose_matrix(angles=np.array([0.0, 0.0, Od]), translate=np.array([0.0, 0.0, 0.0]))

            elif msg1.cmd.type == "straight":
                self.mparams['type'] = "straight" 
                for name, value in dict(zip(msg1.cmd.param_names, msg1.cmd.param_values)).items():
                        if name == "yaw_error" :
                            self.o_yaw -= float(value)
                        elif name != "side" :
                            self.mparams[name] = float(value)
                        else:
                            self.mparams[name] = (value)
                self.mparams['distance'] = 1000000.0
                self.mparams['heading'] = 0.0
                self.mparams['side'] = 'left'

                # Create T_g_s for straight command
                xd = self.mparams['velocity'] * timeout
                self.T_g_s = compose_matrix(angles=np.array([0.0, 0.0, 0.0]), translate=np.array([xd, 0.0, 0.0]))

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
            self.get_logger().debug(self.mparams)
            self.models = model(self.mtime ,self.mparams)
            self.GOAL_REACHED = False


        elif  msg1.cmd.event == 'STOP' :

            self.get_logger().info("In experiment call back, stop")
            self.ACTIVE = True
            self.event = 'STOP'
        
            self.mparams = {}
            self.mtime = 0
            self.mparams['type'] = 'straight'
            self.mparams['distance'] = 100000
            self.mparams['heading'] = 0.0
            self.mparams['velocity'] = 0.0
            self.mparams['side'] = 0.0
            self.get_logger().debug(self.mparams)
            self.models = model(self.mtime ,self.mparams)

            msg = Command()
            msg.id = 0
            msg.start = float(0.0)
            msg.timeout = float(5)
            msg.event = self.event 
            msg.targets =["mobility_base"] 
            msg.type = self.mparams['type']
            msg.param_names = ["distance","velocity","side","heading"]
            msg.param_values = [str(0.0),str(self.mparams['velocity']),str(self.mparams['side']),str(self.mparams['heading'])]
            self.y_publisher.publish(msg)

    def limit(self, value, key):
        return np.clip(value, -self.limits[key], self.limits[key])

    def apply_PI_control(self, y_error, O_error, x_dot, angle):
        # PI controller for orientation drift error

        # Find the limits
        self.gains['kpO'] = 0.25
        self.gains['kiO'] = 0.000
        self.gains['kpy'] = 0.1
        self.gains['kiy'] = 0.000
        omega = (self.gains['kpO'] * O_error + self.gains['kiO'] * self.O_cum_error)
        omegafb = self.limit(omega, 'w')
       
        # PI controller for lateral slip error
        # when y actual > y projected, ydot is negative
        y_dot = (self.gains['kpy'] * y_error + self.gains['kiy'] * self.y_cum_error) #* (-1 if x_dot > 0 else 1)

        # Calculate corrective heading from lateral controller and invoke limit
        heading = math.atan2(y_dot, abs(x_dot))
        headingfb = self.limit(heading, 'h') 

        # Calculate radius, angle, lin_vel, side feedback parameters
        radiusfb, anglefb, velocity, sidefb = self.vel_to_traj(x_dot, self.ang_vel+omegafb, headingfb)
        velocityfb = self.limit(velocity, 'v') 
        return radiusfb, anglefb, velocityfb, sidefb, omegafb, headingfb, y_dot
        
    def periodic(self):
        if not self.ACTIVE or not self.LOC_READY or self.GOAL_REACHED:
            return
        
        ts = self.get_clock().now().to_msg()
        self.curr_time = ts.sec + ts.nanosec * 1e-9
        t = self.curr_time - self.start_time    ####(calculate the time elapsed)

        # Calculate xa, ya, Oa from transform between global and start frame
        # T_s_g = compose_matrix(translate=[self.o_posx, self.o_posy, self.o_posz], angles=[self.o_roll, self.o_pitch, self.o_yaw])
        # T_a_g = compose_matrix(translate=[self.a_posx, self.a_posy, self.a_posz], angles=[self.a_roll, self.a_pitch, self.a_yaw])
        # T_g_s = np.linalg.inv(T_s_g)
        T_c_s = np.linalg.inv(self.T_s_w) @ self.T_c_w
        _, _, angles, trans, _ = decompose_matrix(T_c_s)
        self.rel_x = trans[0]
        self.rel_y = trans[1]
        self.rel_ori = angles[2]

        xa = self.rel_x
        ya = self.rel_y
        Oa = self.rel_ori
        va = self.vel

        # Check if rover reached goal
        self.check_reached_goal(self.mparams, xa, ya, Oa)
        if self.GOAL_REACHED:
            return

        alpha = self.mparams['heading']

        self.mparams['angle'] = (self.mparams['heading'] + math.pi/2) if (self.mparams['side'] == 'left') else (self.mparams['heading'] - math.pi/2) * -1
        _, _, _, _, xp, yp, Op = model(self.mtime,self.mparams)(t, xa, ya, Oa, va)
        
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

        O_error = Op - Oa
        y_error = ypr - yar

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
            y_dot = 0.0

        self.O_cum_error += O_error
        self.y_cum_error += y_error

        if self.ACTIVE:
            log = self.get_logger()
            log.info(f'    xp: {xp}, yp: {yp}, Op: {Op}')
            log.info(f'    xa: {xa}, ya: {ya}, Oa: {Oa}')
            log.info(f'    O_error: {O_error}, y_error: {y_error}')
            log.info(f'    O_cum_error: {self.O_cum_error}, y_cum_error: {self.y_cum_error}')
            log.info(f'    omegafb: {omegafb}, headingfb: {headingfb}, y_dot: {y_dot}')
            log.info(f'    radiusfb: {radiusfb}, anglefb: {anglefb}, velocityfb: {velocityfb}, sidefb: {sidefb}')
            msg = Command()
            msg.id = 0
            msg.start = float(0.0)
            msg.timeout = float(5)
            msg.event = self.event 
            msg.targets =["mobility_base"] 
            msg.type = cmd_type
            msg.param_names = ["distance","velocity","side","heading"]
            msg.param_values = [str(radiusfb),str(velocityfb),str(sidefb),str(headingfb)]
            self.y_publisher.publish(msg)

            debug_msg = SimpleStatus()
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.param_names = ['xp', 'yp', 'Op', 'xa', 'ya', 'Oa', 'O_error', 'y_error', 'O_cum_error', 'y_cum_error', 'omegafb', 'headingfb', 'y_dot', 'radiusfb', 'anglefb', 'velocityfb', 'sidefb', 'ax', 'ay', 'az']
            sidefb_float = 0.0 if sidefb == 'right' else 1.0
            debug_msg.param_values = [float(xp), float(yp), float(Op), float(xa), float(ya), float(Oa), float(O_error), float(y_error[0]), float(self.O_cum_error), float(self.y_cum_error[0]), float(omegafb), float(headingfb), float(y_dot), float(radiusfb), float(anglefb), float(velocityfb), sidefb_float, float(self.a_posx), float(self.a_posy), float(self.a_posz)]
            self.debug_pub.publish(debug_msg)

def main():
    rclpy.init()
    node = SimpleTracker('path_control_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


