import numpy as np
import copy
import math as m
from rover_modules.utils import ros2py
from rover_msgs.msg import *
from tf.transformations import compose_matrix, decompose_matrix, inverse_matrix, euler_matrix


def quat_to_yaw(x, y, z, w):
    yaw = ((m.atan2((2*(w*z + x*y)), 
                      (1 - 2*(y**2 + z**2)))
          ) % (2 * m.pi))
          #* (180 / m.pi)) % 360)
    return yaw


last_description = ""
start_time = 0.0
count = 0
static_sinkage = 0
start_frame = compose_matrix()
target_vel = 0
def reset(global_config, filter_config, pub_msgs, sub_msgs):
    global static_sinkage
    global start_time
    global start_frame
    global last_description
    global last_world_pose
    global last_filt_rover
    global last_filt_rr_vel
    global last_filt_rr_drive_current
    global last_filt_rr_steer_current
    global last_filt_rover
    global last_trench_features 
    global target_vel
    global last_time

    exp_msg = sub_msgs['exp']
    float_time = exp_msg.header.stamp.secs + exp_msg.header.stamp.nsecs * 10 ** -9
    if exp_msg.description != last_description:    
        last_filt_rr_vel = None
        last_filt_rr_drive_current = None
        last_filt_rr_steer_current = None
        last_filt_rover = np.array([])
        last_world_pose = np.array([])
        last_time = None
        last_trench_features = [None, None, None, None, None]
        start_time = float_time
        static_sinkage = sub_msgs['sinkage'].sinkage

        params = ros2py(exp_msg.cmd.param_names, exp_msg.cmd.param_values)
        target_vel = float(params['velocity'])

        g_pose = sub_msgs['global_pose']
        theta = quat_to_yaw(g_pose.pose.pose.orientation.x,
                            g_pose.pose.pose.orientation.y,
                            g_pose.pose.pose.orientation.z,
                            g_pose.pose.pose.orientation.w)
        start_frame = compose_matrix(angles=np.array([0, 0, theta]),
                                     translate=np.array([g_pose.pose.pose.position.x,
                                                         g_pose.pose.pose.position.y, 0]))

    pub_msgs['time_[s]'].data = float_time - start_time
    pub_msgs['exp_desc'].data = exp_msg.description
    last_description = exp_msg.description


def apply_ewma_filter(alpha, current_val, last_val):
    return alpha * current_val + (1 - alpha) * last_val


last_filt_rover = np.array([])
last_world_pose = np.array([])
last_time = None
def extract_world_pose_vel(global_config, filter_config, pub_msgs, sub_msgs):
    # The state estimation nodes of robot_localization produce a state estimate 
    # whose pose is given in the map or odom frame and whose velocity is 
    # given in the base_link frame
    global last_filt_rover
    global last_world_pose
    global last_time

    g_pose = sub_msgs['global_pose']
    time = sub_msgs['global_pose'].header.stamp.to_sec()

    pub_msgs['wr_world_pose_[m]'].x = g_pose.pose.pose.position.x
    pub_msgs['wr_world_pose_[m]'].y = g_pose.pose.pose.position.y
    pub_msgs['wr_world_pose_[m]'].theta = quat_to_yaw(g_pose.pose.pose.orientation.x,
                                                 g_pose.pose.pose.orientation.y,
                                                 g_pose.pose.pose.orientation.z,
                                                 g_pose.pose.pose.orientation.w)

    if last_time:
        world_vel = np.array([(pub_msgs['wr_world_pose_[m]'].x - last_world_pose[0]) / (time - last_time),
                              (pub_msgs['wr_world_pose_[m]'].y - last_world_pose[1]) / (time - last_time), 
                              (pub_msgs['wr_world_pose_[m]'].theta - last_world_pose[2]) / (time - last_time)])
        R = euler_matrix(0, 0, pub_msgs['wr_world_pose_[m]'].theta)

        rover_vel = np.dot(inverse_matrix(R), [world_vel[0], world_vel[1], 0, 1])

        pub_msgs['wr_rover_vel_[m/s]'].x = rover_vel[0]
        pub_msgs['wr_rover_vel_[m/s]'].y = rover_vel[1]
        pub_msgs['wr_rover_vel_[m/s]'].theta = world_vel[2]
    else:                     
        pub_msgs['wr_rover_vel_[m/s]'].x = 0.0
        pub_msgs['wr_rover_vel_[m/s]'].y = 0.0
        pub_msgs['wr_rover_vel_[m/s]'].theta = 0.0 
                              
    if last_filt_rover.size > 0:
        filt_rover_vel_x = apply_ewma_filter(0.01, pub_msgs['wr_rover_vel_[m/s]'].x, last_filt_rover[0])
        filt_rover_vel_y = apply_ewma_filter(0.0025, pub_msgs['wr_rover_vel_[m/s]'].y, last_filt_rover[1])
        filt_rover_vel_ang = apply_ewma_filter(0.0025, pub_msgs['wr_rover_vel_[m/s]'].theta, last_filt_rover[2])
    else:
        filt_rover_vel_x = pub_msgs['wr_rover_vel_[m/s]'].x
        filt_rover_vel_y = pub_msgs['wr_rover_vel_[m/s]'].y
        filt_rover_vel_ang = pub_msgs['wr_rover_vel_[m/s]'].theta

    pub_msgs['wr_rover_filt_vel_[m/s]'].x = filt_rover_vel_x
    pub_msgs['wr_rover_filt_vel_[m/s]'].y = filt_rover_vel_y
    pub_msgs['wr_rover_filt_vel_[m/s]'].theta = filt_rover_vel_ang
    last_filt_rover = np.array([filt_rover_vel_x, filt_rover_vel_y, filt_rover_vel_ang])
    last_world_pose = np.array([pub_msgs['wr_world_pose_[m]'].x,
                                pub_msgs['wr_world_pose_[m]'].y,
                                pub_msgs['wr_world_pose_[m]'].theta])
    last_time = time


def calc_ang_to_lin_vel(ang_vel, pos):
    if ang_vel > 0:
        vel = ang_vel * np.array([-pos[1], pos[0]])
    else:
        vel = ang_vel * np.array([pos[1], -pos[0]])
    return vel


last_filt_rr_vel = None
def extract_rover_pose_vel(global_config, filter_config, pub_msgs, sub_msgs):
    global start_frame
    global last_filt_rr_vel

    w_pose = pub_msgs['wr_world_pose_[m]']
    r_vel = pub_msgs['wr_rover_vel_[m/s]']
    r_filt_vel = pub_msgs['wr_rover_filt_vel_[m/s]']
    positions = dict(zip(sub_msgs['rover_state'].name, sub_msgs['rover_state'].position))
    velocities = dict(zip(sub_msgs['rover_state'].name, sub_msgs['rover_state'].velocity))

    w_frame = compose_matrix(angles=np.array([0, 0, w_pose.theta]),
                             translate=np.array([w_pose.x, w_pose.y, 0]))

    rel_frame = np.dot(inverse_matrix(start_frame), w_frame)

    _, _, pose_angles, pose_trans, _ = decompose_matrix(rel_frame)
    pub_msgs['wr_start_pose_[m]'].x = pose_trans[0]
    pub_msgs['wr_start_pose_[m]'].y = pose_trans[1]
    pub_msgs['wr_start_pose_[m]'].theta = pose_angles[2]

    rr_pos_rel_baselink = np.array([-0.186, -0.144])

    rr_actual_vel = np.array([r_vel.x, r_vel.y]) + calc_ang_to_lin_vel(r_vel.theta, rr_pos_rel_baselink)
    pub_msgs['rr_actual_vel_[m/s]'].x = rr_actual_vel[0]
    pub_msgs['rr_actual_vel_[m/s]'].y = rr_actual_vel[1]
    pub_msgs['rr_actual_vel_[m/s]'].theta = r_vel.theta

    rr_actual_filt_vel = np.array([r_filt_vel.x, r_filt_vel.y]) + calc_ang_to_lin_vel(r_filt_vel.theta, rr_pos_rel_baselink)
    pub_msgs['rr_actual_filt_vel_[m/s]'].x = rr_actual_filt_vel[0]
    pub_msgs['rr_actual_filt_vel_[m/s]'].y = rr_actual_filt_vel[1]
    pub_msgs['rr_actual_filt_vel_[m/s]'].theta = r_filt_vel.theta

    rr_vel = velocities['RR/driving'] * global_config['measures']['wheel_radii']['RR']
    if last_filt_rr_vel is not None:
        rr_filt_vel = apply_ewma_filter(0.09, rr_vel, last_filt_rr_vel)
    else:
        rr_filt_vel = rr_vel

    trench_angle = positions['RR/steering']

    pub_msgs['rr_vel_[m/s]'].x = rr_vel * m.cos(trench_angle)
    pub_msgs['rr_vel_[m/s]'].y = rr_vel * m.sin(trench_angle)
    pub_msgs['rr_vel_[m/s]'].theta = 0

    pub_msgs['rr_filt_vel_[m/s]'].x = rr_filt_vel * m.cos(trench_angle)
    pub_msgs['rr_filt_vel_[m/s]'].y = rr_filt_vel * m.sin(trench_angle)
    pub_msgs['rr_filt_vel_[m/s]'].theta = 0   
    
    pub_msgs['rr_trench_angle_[rad]'].data = trench_angle

    last_filt_rr_vel = rr_filt_vel


def calc_wheel_slip(rw, vx):
    thresh = 0.001
    if vx < rw:
        slip = (rw - vx) / rw if abs(rw) > thresh else 0
    else:
        slip = (rw - vx) / vx if abs(vx) > thresh else 0
    return slip


def extract_slip(global_config, filter_config, pub_msgs, sub_msgs):
    rr_filt_vel = pub_msgs['rr_filt_vel_[m/s]']
    rr_act_filt_vel = pub_msgs['rr_actual_filt_vel_[m/s]']
    rr_trench_angle = pub_msgs['rr_trench_angle_[rad]']

    rr_act_slip = calc_wheel_slip(rr_filt_vel.x, rr_act_filt_vel.x)
    rr_proj_slip = calc_wheel_slip(rr_filt_vel.x, target_vel)

    pub_msgs['rr_proj_slip'].data = rr_proj_slip
    pub_msgs['rr_act_slip'].data = rr_act_slip
    pub_msgs['rr_slip_angle_[rad]'].data = rr_trench_angle.data - m.atan2(rr_act_filt_vel.y, rr_act_filt_vel.x)


def calc_slip_sinkage(global_config, filter_config, pub_msgs, sub_msgs):
    global static_sinkage

    proj_rr_slip = pub_msgs['rr_proj_slip'].data
    act_rr_slip = pub_msgs['rr_act_slip'].data

    pub_msgs['rr_proj_slip_sinkage_[m]'].data = (1 + proj_rr_slip) / \
                                            (1 - 0.5 * proj_rr_slip) * static_sinkage

    pub_msgs['rr_act_slip_sinkage_[m]'].data = (1 + act_rr_slip) / \
                                           (1 - 0.5 * act_rr_slip) * static_sinkage


last_trench_features = [None, None, None, None, None]
def extract_trench_features(global_config, filter_config, pub_msgs, sub_msgs):
    global last_trench_features

    trench = sub_msgs['trench_features'] 

    for feature in trench.feature_list:
        section_no = feature.section_no

        pub_feature = FeatureMsg()
        pub_feature.section_no = section_no
        pub_feature.slope1 = feature.slope1 / 1000.0
        pub_feature.slope2 = feature.slope2 / 1000.0
        pub_feature.slope3 = feature.slope3 / 1000.0
        pub_feature.top_width = feature.top_width / 1000.0
        pub_feature.bottom_width = feature.bottom_width / 1000.0
        pub_feature.pile_height = feature.pile_height / 1000.0
        pub_feature.trench_depth = feature.trench_depth / 1000.0

        pub_msgs['section_' + str(section_no) + '_bool'] = True
        pub_msgs['section_' + str(section_no) + '_features'] = pub_feature

        last_trench_features[section_no] = pub_feature

    for ctr, valid in enumerate(trench.boolean_list):
        if not valid:
            if not last_trench_features[ctr]:
                pub_feature = FeatureMsg()
                pub_feature.section_no = ctr
                pub_feature.slope1 = 0
                pub_feature.slope2 = 0
                pub_feature.slope3 = 0
                pub_feature.top_width = 0
                pub_feature.bottom_width = 0
                pub_feature.pile_height = 0
                pub_feature.trench_depth = 0 
            else: 
                pub_feature = last_trench_features[ctr]
            pub_msgs['section_' + str(ctr) + '_bool'] = False
            pub_msgs['section_' + str(ctr) + '_features'] = pub_feature 
            last_trench_features[ctr] = pub_feature


last_filt_rr_drive_current = None
last_filt_rr_steer_current = None
def remap_current(global_config, filter_config, pub_msgs, sub_msgs): 
    global last_filt_rr_drive_current
    global last_filt_rr_steer_current

    joints = sub_msgs['rover_state']
    status = sub_msgs['rover_status']
    pub_msgs['joints'].effort = [0 for i in range(len(joints.name))]

    for motor in status.sources:
        idx = joints.name.index(motor)
        current_idx = status.param_names[idx].data.index('current')
        pub_msgs['joints'].effort[idx] = status.param_values[idx].data[current_idx]
        if motor == 'RR/driving':
            if last_filt_rr_drive_current is not None:
                drive_filt_current = apply_ewma_filter(0.01,
                                                       float(status.param_values[idx].data[current_idx]),
                                                       last_filt_rr_drive_current)
            else:
                drive_filt_current = float(status.param_values[idx].data[current_idx])
        if motor == 'RR/steering':
            if last_filt_rr_steer_current is not None:
                steer_filt_current = apply_ewma_filter(0.01,
                                                       float(status.param_values[idx].data[current_idx]),
                                                       last_filt_rr_steer_current)
            else:
                steer_filt_current = float(status.param_values[idx].data[current_idx])

    pub_msgs['joints'].name = joints.name
    pub_msgs['joints'].velocity = joints.velocity
    pub_msgs['joints'].position = joints.position
    pub_msgs['rr_drive_current_[A]'].data = drive_filt_current
    pub_msgs['rr_steer_current_[A]'].data = steer_filt_current

    last_filt_rr_drive_current = drive_filt_current
    last_filt_rr_steer_current = steer_filt_current


def map_sinkage(global_config, filter_config, pub_msgs, sub_msgs):
    pub_msgs['sinkage_[m]'].data = sub_msgs['sinkage'].sinkage
