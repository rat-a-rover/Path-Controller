import rospy
import math
import numpy as np
from rover_modules.utils import ros2py, py2ros_nostr
from rover_msgs.msg import SimpleStatus, RiskMetric
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


def degs_to_ms(vel_deg_s, radius):
    return vel_deg_s * math.pi / 180 * radius


def deg_to_m(pos_deg, radius):
    return pos_deg * math.pi / 180 * radius


def ewma_filter(alpha, current_val, last_val):
    return alpha * current_val + (1 - alpha) * last_val


def window_filter(filter, window, new_data, data_array):
    if data_array.shape[0] < window:
        data_array = (np.vstack((new_data, data_array))
                         if data_array.size
                         else np.array([new_data]))
    else:
        data_array = np.roll(data_array, 1, axis=0)
        data_array[0] = new_data

    filtered_data = (filter(data_array, axis=0)
                     if len(data_array.shape) > 1
                     else data_array)
    return filtered_data[0], data_array


def get_joint_state(rad, vel, pos, tq):
    js_msg = JointState()
    js_msg.name = ['base_link']
    js_msg.velocity = [degs_to_ms(vel, rad)]
    js_msg.position = [deg_to_m(pos, rad)]
    js_msg.effort = [tq]
    return js_msg


def get_odom(lin_vel):
    msg = Odometry()
    msg.child_frame_id = 'base_link'
    msg.header.frame_id = 'joint_odom'
    msg.twist.twist.linear.x = lin_vel
    msg.twist.covariance = [4e-8, 0, 0, 0, 0, 0,
                            0, 4e-8, 0, 0, 0, 0, 
                            0, 0, 4e-8, 0, 0, 0,
                            0, 0, 0, 4e-8, 0, 0,
                            0, 0, 0, 0, 4e-8, 0,
                            0, 0, 0, 0, 0, 4e-8]
    return msg


def calc_slip(wheel_lin_vel, actual_lin_vel):
    slip_vel_thresh = .005

    if abs(wheel_lin_vel) <= slip_vel_thresh:
        slip = -1
    else:
        slip = (wheel_lin_vel - actual_lin_vel) / wheel_lin_vel * 100

    if slip == -1:
        slip_class = 0
    elif slip < 20: 
        slip_class = 1
    elif slip < 70:
        slip_class = 2
    else:
        slip_class = 3

    return slip, slip_class 


def get_slip(wheel_lin_vel, testbed_lin_vel, slip_array):
    slip, slip_class = calc_slip(wheel_lin_vel, testbed_lin_vel)
    avg_slip_class = window_filter(np.median, 10, slip_class, slip_array)
    slip_msg = RiskMetric()
    slip_msg.name = "Slip"
    slip_msg.data_names = ['Slip Class', 'Average Slip Class', 'Slip', 'Wheel Vel', 'Carriage Vel']
    slip_msg.data_values = [slip_class, avg_slip_class[0], slip, wheel_lin_vel, testbed_lin_vel]

    return slip_msg, slip_array

#ewma_alpha = 1
#last_embedding_rate = float('inf')
sinkage_filt_window = 50
embedding_arr = np.array([[]])
sinkage_arr = np.array([[]])
def get_embedding(rad, embedding_data, sinkage_data):
    #global ewma_alpha
    #global last_embedding_rate
    global filt_window
    global embedding_arr
    global sinkage_arr
    sinkage_offset, sinkage_arr = window_filter(np.median, sinkage_filt_window, 
                                                sinkage_data, sinkage_arr)
    sinkage = sinkage_offset - rad
    embedding_rate, embedding_arr = window_filter(np.median, sinkage_filt_window,
                                                  embedding_data, embedding_arr)

    embed_msg = SimpleStatus()
    embed_msg.name = "Testbed Embed" 
    embed_msg.source = "Wheel Test Bed"
    embed_msg.type = "Processed Sensor Data"
    embed_msg.param_names = ['Sinkage', 'Embedding Rate']
    embed_msg.param_values = [sinkage, embedding_rate]

    return embed_msg

'''
def process_testbed_sensors(global_config, filter_config, pub_msgs, sub_msgs):
    testbed_sensor_states = ros2py(sub_msgs['testbed_msg'].param_names, 
                                    sub_msgs['testbed_msg'].param_values)

    pub_msgs['joint_state_msg'] = get_joint_state(global_config['wheel_radius'],
                                                  testbed_sensor_states['WHEEL_MC_FB_VEL'],
                                                  testbed_sensor_states['WHEEL_MC_FB_POS'],
                                                  testbed_sensor_states['WHEEL_ALL_TQ_ACTUAL'])

    pub_msgs['odom_msg'] = get_odom(pub_msgs['joint_state_msg'].velocity[0])

    pub_msgs['slip_msg'] = get_slip(pub_msgs['joint_state_msg'].velocity[0],
                                    testbed_sensor_states['CARRIAGE_H_VEL'])

    pub_msgs['embed_msg'] = get_embedding(global_config['wheel_radius'],
                                          testbed_sensor_states['CARRIAGE_V_VEL_SOIL'],
                                          testbed_sensor_states['CARRIAGE_V_HEIGHT_SOIL_ACTUAL'])
'''

def inv_weighted_average(vals, inv_wts):
    default_nan_wt = 999
    np_vals = np.array(vals)
    wts = 1.0 / np.array([inv_wt if inv_wt != 0 else default_nan_wt for inv_wt in inv_wts]) 
    return np.dot(wts, np_vals) / np.sum(wts)


vo_odom_filt_window = 7
vo_odom_lin_x_array = np.array([[]])
vo_odom_lin_y_array = np.array([[]])
vo_odom_lin_z_array = np.array([[]])
vo_odom_ang_x_array = np.array([[]])
vo_odom_ang_y_array = np.array([[]])
vo_odom_ang_z_array = np.array([[]])
def filter_fused_vo_odom(global_config, filter_config, pub_msgs, sub_msgs):
    global vo_odom_filt_window
    global vo_odom_lin_x_array
    global vo_odom_lin_y_array
    global vo_odom_lin_z_array
    global vo_odom_ang_x_array
    global vo_odom_ang_y_array
    global vo_odom_ang_z_array
    
    twist = sub_msgs['sub_icp_fused_msg'].twist
    
    x_lin_vel, vo_odom_lin_x_array = window_filter(np.median, vo_odom_filt_window, 
                                                   twist.twist.linear.x, vo_odom_lin_x_array)
    y_lin_vel, vo_odom_lin_y_array = window_filter(np.median, vo_odom_filt_window, 
                                                   twist.twist.linear.y, vo_odom_lin_y_array)
    z_lin_vel, vo_odom_lin_z_array = window_filter(np.median, vo_odom_filt_window, 
                                                   twist.twist.linear.z, vo_odom_lin_z_array)
    x_ang_vel, vo_odom_ang_x_array = window_filter(np.median, vo_odom_filt_window, 
                                                   twist.twist.angular.x, vo_odom_ang_x_array)
    y_ang_vel, vo_odom_ang_y_array = window_filter(np.median, vo_odom_filt_window, 
                                                   twist.twist.angular.y, vo_odom_ang_y_array)
    z_ang_vel, vo_odom_ang_z_array = window_filter(np.median, vo_odom_filt_window, 
                                                   twist.twist.angular.z, vo_odom_ang_z_array)

    pub_msgs['pub_vo_odom_msg'].header.stamp = rospy.Time.now()
    pub_msgs['pub_vo_odom_msg'].header.frame_id = 'odom'
    pub_msgs['pub_vo_odom_msg'].child_frame_id = 'base_link'
    pub_msgs['pub_vo_odom_msg'].twist.twist.linear.x = x_lin_vel
    pub_msgs['pub_vo_odom_msg'].twist.twist.linear.y = y_lin_vel
    pub_msgs['pub_vo_odom_msg'].twist.twist.linear.z = z_lin_vel
    pub_msgs['pub_vo_odom_msg'].twist.twist.angular.x = x_ang_vel
    pub_msgs['pub_vo_odom_msg'].twist.twist.angular.y = y_ang_vel
    pub_msgs['pub_vo_odom_msg'].twist.twist.angular.z = z_ang_vel
    pub_msgs['pub_vo_odom_msg'].twist.covariance = sub_msgs['sub_icp_fused_msg'].twist.covariance 
    

def fuse_covariance(covariance_arrays):
    default_nan_val = 999
    sum_inv_cov = 0
    for cov_array in covariance_arrays:
        stripped_array = [cov_array[0], cov_array[7], cov_array[14], cov_array[21], cov_array[28], cov_array[35]]
        sum_inv_cov += 1.0 / (np.array([val if val != 0 else default_nan_val for val in stripped_array]))
    cov = sum_inv_cov ** -0.5
    return [cov[0],      0,      0,      0,      0,     0, 
                 0, cov[1],      0,      0,      0,     0, 
                 0,      0, cov[2],      0,      0,     0,
                 0,      0,      0, cov[3],      0,     0,
                 0,      0,      0,      0, cov[4],     0,
                 0,      0,      0,      0,      0, cov[5]]


def fuse_vo_odom(global_config, filter_config, pub_msgs, sub_msgs):
    twist_f = sub_msgs['sub_front_vo_odom_msg'].twist
    twist_s = sub_msgs['sub_side_vo_odom_msg'].twist
    twist_fg = sub_msgs['sub_front_guess_vo_odom_msg'].twist
    twist_sg = sub_msgs['sub_side_guess_vo_odom_msg'].twist

    
    raw_x_lin_vel_fs = inv_weighted_average([twist_f.twist.linear.x, twist_s.twist.linear.x],
                                            [twist_f.covariance[0], twist_s.covariance[0]])

    raw_y_lin_vel_fs = inv_weighted_average([twist_f.twist.linear.y, twist_s.twist.linear.y],
                                            [twist_f.covariance[7], twist_s.covariance[7]]) 

    raw_z_lin_vel_fs = inv_weighted_average([twist_f.twist.linear.z, twist_s.twist.linear.z],
                                            [twist_f.covariance[14], twist_s.covariance[14]])

    raw_x_ang_vel_fs = inv_weighted_average([twist_f.twist.angular.x, twist_s.twist.angular.x],
                                            [twist_f.covariance[21], twist_s.covariance[21]])

    raw_y_ang_vel_fs = inv_weighted_average([twist_f.twist.angular.y, twist_s.twist.angular.y],
                                            [twist_f.covariance[28], twist_s.covariance[28]])

    raw_z_ang_vel_fs = inv_weighted_average([twist_f.twist.angular.z, twist_s.twist.angular.z],
                                            [twist_f.covariance[35], twist_s.covariance[35]])

    cov_fs = fuse_covariance([twist_f.covariance, twist_s.covariance])

    raw_x_lin_vel_fgsg = inv_weighted_average([twist_fg.twist.linear.x, twist_sg.twist.linear.x],
                                              [twist_fg.covariance[0], twist_sg.covariance[0]])

    raw_y_lin_vel_fgsg = inv_weighted_average([twist_fg.twist.linear.y, twist_sg.twist.linear.y],
                                              [twist_fg.covariance[7], twist_sg.covariance[7]]) 

    raw_z_lin_vel_fgsg = inv_weighted_average([twist_fg.twist.linear.z, twist_sg.twist.linear.z],
                                              [twist_fg.covariance[14], twist_sg.covariance[14]])

    raw_x_ang_vel_fgsg = inv_weighted_average([twist_fg.twist.angular.x, twist_sg.twist.angular.x],
                                              [twist_fg.covariance[21], twist_sg.covariance[21]])

    raw_y_ang_vel_fgsg = inv_weighted_average([twist_fg.twist.angular.y, twist_sg.twist.angular.y],
                                              [twist_fg.covariance[28], twist_sg.covariance[28]])

    raw_z_ang_vel_fgsg = inv_weighted_average([twist_fg.twist.angular.z, twist_sg.twist.angular.z],
                                              [twist_fg.covariance[35], twist_sg.covariance[35]])

    cov_fgsg = fuse_covariance([twist_fg.covariance, twist_sg.covariance])

    raw_x_lin_vel_all = inv_weighted_average([raw_x_lin_vel_fs, raw_x_lin_vel_fgsg], [cov_fs[0],  cov_fgsg[0]])
    raw_y_lin_vel_all = inv_weighted_average([raw_y_lin_vel_fs, raw_y_lin_vel_fgsg], [cov_fs[7],  cov_fgsg[7]])
    raw_z_lin_vel_all = inv_weighted_average([raw_y_lin_vel_fs, raw_y_lin_vel_fgsg], [cov_fs[14], cov_fgsg[14]])
    raw_x_ang_vel_all = inv_weighted_average([raw_x_ang_vel_fs, raw_x_ang_vel_fgsg], [cov_fs[21], cov_fgsg[21]])
    raw_y_ang_vel_all = inv_weighted_average([raw_y_ang_vel_fs, raw_y_ang_vel_fgsg], [cov_fs[28], cov_fgsg[28]])
    raw_z_ang_vel_all = inv_weighted_average([raw_y_ang_vel_fs, raw_y_ang_vel_fgsg], [cov_fs[35], cov_fgsg[35]])

    cov_all = fuse_covariance([cov_fs, cov_fgsg])
    
    time_now = rospy.Time.now()

    pub_msgs['pub_vo_odom_fs_msg'].header.stamp = time_now
    pub_msgs['pub_vo_odom_fs_msg'].header.frame_id = 'fuse_odom_fs'
    pub_msgs['pub_vo_odom_fs_msg'].child_frame_id = 'base_link'
    pub_msgs['pub_vo_odom_fs_msg'].twist.twist.linear.x = raw_x_lin_vel_fs
    pub_msgs['pub_vo_odom_fs_msg'].twist.twist.linear.y = raw_y_lin_vel_fs
    pub_msgs['pub_vo_odom_fs_msg'].twist.twist.linear.z = raw_z_lin_vel_fs
    pub_msgs['pub_vo_odom_fs_msg'].twist.twist.angular.x = raw_x_ang_vel_fs
    pub_msgs['pub_vo_odom_fs_msg'].twist.twist.angular.y = raw_y_ang_vel_fs
    pub_msgs['pub_vo_odom_fs_msg'].twist.twist.angular.z = raw_z_ang_vel_fs
    pub_msgs['pub_vo_odom_fs_msg'].twist.covariance = cov_fs

    pub_msgs['pub_vo_odom_fgsg_msg'].header.stamp = time_now
    pub_msgs['pub_vo_odom_fgsg_msg'].header.frame_id = 'fuse_odom_fgsg'
    pub_msgs['pub_vo_odom_fgsg_msg'].child_frame_id = 'base_link'
    pub_msgs['pub_vo_odom_fgsg_msg'].twist.twist.linear.x = raw_x_lin_vel_fgsg
    pub_msgs['pub_vo_odom_fgsg_msg'].twist.twist.linear.y = raw_y_lin_vel_fgsg
    pub_msgs['pub_vo_odom_fgsg_msg'].twist.twist.linear.z = raw_z_lin_vel_fgsg
    pub_msgs['pub_vo_odom_fgsg_msg'].twist.twist.angular.x = raw_x_ang_vel_fgsg
    pub_msgs['pub_vo_odom_fgsg_msg'].twist.twist.angular.y = raw_y_ang_vel_fgsg
    pub_msgs['pub_vo_odom_fgsg_msg'].twist.twist.angular.z = raw_z_ang_vel_fgsg
    pub_msgs['pub_vo_odom_fgsg_msg'].twist.covariance = cov_fgsg

    pub_msgs['pub_vo_odom_all_msg'].header.stamp = time_now
    pub_msgs['pub_vo_odom_all_msg'].header.frame_id = 'fuse_odom_all'
    pub_msgs['pub_vo_odom_all_msg'].child_frame_id = 'base_link'
    pub_msgs['pub_vo_odom_all_msg'].twist.twist.linear.x = raw_x_lin_vel_all
    pub_msgs['pub_vo_odom_all_msg'].twist.twist.linear.y = raw_y_lin_vel_all
    pub_msgs['pub_vo_odom_all_msg'].twist.twist.linear.z = raw_z_lin_vel_all
    pub_msgs['pub_vo_odom_all_msg'].twist.twist.angular.x = raw_x_ang_vel_all
    pub_msgs['pub_vo_odom_all_msg'].twist.twist.angular.y = raw_y_ang_vel_all
    pub_msgs['pub_vo_odom_all_msg'].twist.twist.angular.z = raw_z_ang_vel_all
    pub_msgs['pub_vo_odom_all_msg'].twist.covariance = cov_all


imu_ewma_alpha = .1
last_imu_msg = None
def filter_imu(imu_msg):
    global last_imu_msg
    global imu_ewma_alpha 

    if last_imu_msg:
        imu_msg.linear_acceleration.x = ewma_filter(imu_ewma_alpha,
                                                    imu_msg.linear_acceleration.x,
                                                    last_imu_msg.linear_acceleration.x)
        imu_msg.linear_acceleration.y = ewma_filter(imu_ewma_alpha,
                                                    imu_msg.linear_acceleration.y,
                                                    last_imu_msg.linear_acceleration.y)
        imu_msg.linear_acceleration.z = ewma_filter(imu_ewma_alpha, 
                                                    imu_msg.linear_acceleration.z,
                                                    last_imu_msg.linear_acceleration.z)
        imu_msg.angular_velocity.x = ewma_filter(imu_ewma_alpha,
                                                 imu_msg.angular_velocity.x,
                                                 last_imu_msg.angular_velocity.x)
        imu_msg.angular_velocity.y = ewma_filter(imu_ewma_alpha,
                                                 imu_msg.angular_velocity.y,
                                                 last_imu_msg.angular_velocity.y)
        imu_msg.angular_velocity.z = ewma_filter(imu_ewma_alpha, 
                                                 imu_msg.angular_velocity.z,
                                                 last_imu_msg.angular_velocity.z)
    else:
        last_imu_msg = imu_msg
    return imu_msg


imu_filt_window = 10
lin_acc_arr = np.array([[]])
def extract_imu_features(imu_msg):
    global lin_acc_arr
    global imu_filt_window
    lin_acc = imu_msg.linear_acceleration.x
    if lin_acc_arr.shape[0] < imu_filt_window:
        lin_acc_arr = (np.vstack((lin_acc, lin_acc_arr))
                         if lin_acc_arr.size
                         else np.array([lin_acc]))
    else:
        lin_acc_arr = np.roll(lin_acc_arr, 1, axis=0)
        lin_acc_arr[0] = lin_acc
    lin_acc_var = np.var(lin_acc_arr, axis=0)
    return lin_acc_var
    

def process_prop_slip_features(global_config, filter_config, pub_msgs, sub_msgs):
    #TODO validate these features, especially imu orientation see config file
    # alternatively, pass all imu data as features and don't worry about orientation
    #also remove testing modifications
    #Check this fact: x_imu = -x_base_link, y_imu = -y_base_link, z_imu = -z_base_link
    testbed_sensor_states = ros2py(sub_msgs['testbed_msg'].param_names, 
                                    sub_msgs['testbed_msg'].param_values)

    wheel_torque =  testbed_sensor_states['WHEEL_ALL_TQ_ACTUAL']
    wheel_lin_vel =  pub_msgs['joint_state_msg'].velocity[0]
    x_lin_acc_var = extract_imu_features(sub_msgs['imu_msg'])
    pub_msgs['imu_msg'] = filter_imu(sub_msgs['imu_msg'])
    x_lin_acc = pub_msgs['imu_msg'].linear_acceleration.x
    y_lin_acc = pub_msgs['imu_msg'].linear_acceleration.y
    z_lin_acc = pub_msgs['imu_msg'].linear_acceleration.z
    x_ang_vel = pub_msgs['imu_msg'].angular_velocity.x
    y_ang_vel = pub_msgs['imu_msg'].angular_velocity.y
    z_ang_vel = pub_msgs['imu_msg'].angular_velocity.z

    prop_features_msg = SimpleStatus()
    prop_features_msg.name = "Proprioceptive Slip Features"
    prop_features_msg.source = "testbed"
    prop_features_msg.type = "processed sensor data"
    prop_features_msg.param_names,\
    prop_features_msg.param_values = py2ros_nostr({'Wheel Torque': wheel_torque,
                                                   'Wheel Linear Velocity': wheel_lin_vel,
                                                   'X Moving Variance'    : x_lin_acc_var,
                                                   'X Linear Acceleration': x_lin_acc,
                                                   'Y Linear Acceleration': y_lin_acc,
                                                   'Z Linear Acceleration': z_lin_acc,
                                                   'X Angular Velocity': x_ang_vel,
                                                   'Y Angular Velocity': y_ang_vel,
                                                   'Z Angular Velocity': z_ang_vel
                                                    })

    pub_msgs['prop_features_msg'] = prop_features_msg

# exp_number: (smooth=0 tw=1, weight in kg)
# Legacy for first grid experiments with no embedded information in description
# training_meta_data = {'01': (0, 50),
#                       '02': (0, 50),
#                       '03': (0, 100),
#                       '04': (0, 100),
#                       '05': (1, 50),
#                       '06': (1, 50),
#                       '07': (1, 100),
#                       '10': (1, 50),
#                       '11': (1, 50),
#                       '14': (0, 50),
#                       '15': (0, 50),
#                       '16': (0, 100),
#                       '17': (0, 100)}
def parse_experiment(description):
    #global training_meta_data
    #wheel_type, weight = training_meta_data[description]
    wheel_type_idx = 1
    weight_idx = 2
    parsed_description = description.split('_')
    wheel_type = int(parsed_description[wheel_type_idx])
    weight = float(parsed_description[weight_idx])
    radius = .30716 if wheel_type == 0 else .2896
    return wheel_type, weight, radius

imu_x_filt_window = 10
imu_z_filt_window = 10
wheel_lin_vel_filt_window = 10
wheel_torque_filt_window = 10

vo_x_filt_window = 7

imu_x_var_array = np.array([[]])
imu_z_var_array = np.array([[]])
wheel_lin_vel_var_array = np.array([[]])
wheel_torque_var_array = np.array([[]])

imu_x_mean_array = np.array([[]])
imu_z_mean_array = np.array([[]])
wheel_lin_vel_mean_array = np.array([[]])
wheel_torque_mean_array = np.array([[]])

vo_x_med_array = np.array([[]])
def process_slip_features(global_config, filter_config, pub_msgs, sub_msgs):
    global imu_x_filt_window
    global imu_z_filt_window
    global wheel_lin_vel_filt_window
    global wheel_torque_filt_window
    global vo_x_filt_window 
    global imu_x_var_array 
    global imu_z_var_array 
    global wheel_lin_vel_var_array
    global wheel_torque_var_array
    global imu_x_mean_array
    global imu_z_mean_array
    global wheel_lin_vel_mean_array
    global wheel_torque_mean_array
    global vo_x_med_array
    testbed_sensor_states = ros2py(sub_msgs['sub_testbed_data'].param_names, 
                                   sub_msgs['sub_testbed_data'].param_values)
    exp_data = parse_experiment(sub_msgs['train_exp_msg'].description)
    imu_msg = sub_msgs['sub_imu']
    twist_msg = sub_msgs['sub_visual_odom'].twist
    testbed_vel = testbed_sensor_states['CARRIAGE_H_VEL']

    wheel_type                       = exp_data[0]
    weight                           = exp_data[1]
    x_lin_acc                        = imu_msg.linear_acceleration.x
    x_lin_acc_var, imu_x_var_array   = window_filter(np.var, imu_x_filt_window, x_lin_acc, imu_x_var_array)
    x_lin_acc_mean, imu_x_mean_array = window_filter(np.mean, imu_x_filt_window, x_lin_acc, imu_x_mean_array)
    z_lin_acc                        = imu_msg.linear_acceleration.z
    z_lin_acc_var, imu_z_var_array   = window_filter(np.var, imu_z_filt_window, z_lin_acc, imu_z_var_array)
    z_lin_acc_mean, imu_z_mean_array = window_filter(np.mean, imu_z_filt_window, z_lin_acc, imu_z_mean_array)
    wheel_torque                     = testbed_sensor_states['WHEEL_ALL_TQ_ACTUAL']
    wheel_torque_var, \
    wheel_torque_var_array          = window_filter(np.var, wheel_torque_filt_window, wheel_torque, wheel_torque_var_array)
    wheel_torque_mean, \
    wheel_torque_mean_array         = window_filter(np.mean, wheel_torque_filt_window, wheel_torque, wheel_torque_mean_array) 
    wheel_lin_vel                    = degs_to_ms(testbed_sensor_states['WHEEL_MC_FB_VEL'], exp_data[2])
    wheel_lin_vel_var, \
    wheel_lin_vel_var_array          = window_filter(np.var, wheel_lin_vel_filt_window, wheel_lin_vel, wheel_lin_vel_var_array)
    wheel_lin_vel_mean, \
    wheel_lin_vel_mean_array         = window_filter(np.mean, wheel_lin_vel_filt_window, wheel_lin_vel, wheel_lin_vel_mean_array) 
    vo_x_vel                         = twist_msg.twist.linear.x
    vo_x_vel_med, vo_x_med_array     = window_filter(np.median, vo_x_filt_window, vo_x_vel, vo_x_med_array)
    vo_x_vel_var                     = twist_msg.covariance[0]
    slip, slip_class                 = calc_slip(wheel_lin_vel, vo_x_vel_med)
    gt_slip, gt_slip_class           = calc_slip(wheel_lin_vel, testbed_vel)

    slip_features_msg = SimpleStatus()
    slip_features_msg.name = "Slip Features"
    slip_features_msg.source = "slip sensors"
    slip_features_msg.type = "slip feature data"
    slip_features_msg.param_names,\
    slip_features_msg.param_values = py2ros_nostr({'Wheel Type'                     : wheel_type,
                                                   'Weight'                         : weight,
                                                   'X Linear Accel'                 : x_lin_acc,
                                                   'X Linear Accel Moving Variance' : x_lin_acc_var,
                                                   'X Linear Accel Mean'            : x_lin_acc_mean,
                                                   'Z Linear Accel'                 : z_lin_acc,
                                                   'Z Linear Accel Moving Variance' : z_lin_acc_var,
                                                   'Z Linear Accel Mean'            : z_lin_acc_mean,
                                                   'Wheel Torque'                   : wheel_torque,
                                                   'Wheel Torque Variance'      : wheel_torque_var,
                                                   'Wheel Torque Mean'          : wheel_torque_mean,
                                                   'Wheel Linear Vel'               : wheel_lin_vel,
                                                   'Wheel Linear Vel Variance'      : wheel_lin_vel_var,
                                                   'Wheel Linear Vel Mean'          : wheel_lin_vel_mean,
                                                   'VO X Linear Vel'                : vo_x_vel,
                                                   'VO X Linear Vel Median'         : vo_x_vel_med,
                                                   'VO X Linear Vel Variance'       : vo_x_vel_var,
                                                   'VO Calculated Slip'             : slip,
                                                   'VO Calculated Slip Class'       : slip_class,
                                                   'GT Calculated Slip'             : gt_slip,
                                                   'GT Calculated Slip Class'       : gt_slip_class
                                                    })
    pub_msgs['pub_slip_features'] = slip_features_msg

gt_slip_array = np.array([[]])
def process_slip_ground_truth(global_config, filter_config, pub_msgs, sub_msgs):
    global gt_slip_array
    testbed_sensor_states = ros2py(sub_msgs['sub_testbed_data'].param_names, 
                                    sub_msgs['sub_testbed_data'].param_values)
    
    exp_data = parse_experiment(sub_msgs['train_exp_msg'].description)
    wheel_lin_vel = degs_to_ms(testbed_sensor_states['WHEEL_MC_FB_VEL'], exp_data[2])
    testbed_lin_vel = testbed_sensor_states['CARRIAGE_H_VEL']

    pub_msgs['pub_gt_slip'], gt_slip_array = get_slip(wheel_lin_vel, testbed_lin_vel, gt_slip_array)


last_feature_sinkage_exp_id = None
LT_baseline_feature_sinkage = 0
LB_baseline_feature_sinkage = 0
MT_baseline_feature_sinkage = 0
MB_baseline_feature_sinkage = 0
RT_baseline_feature_sinkage = 0
RB_baseline_feature_sinkage = 0
sinkage_feature_filt_window = 6
sinkage_feature_array = np.array([[]])
def process_sinkage_features(global_config, filter_config, pub_msgs, sub_msgs):
    global last_feature_sinkage_exp_id
    global LT_baseline_feature_sinkage
    global LB_baseline_feature_sinkage
    global MT_baseline_feature_sinkage
    global MB_baseline_feature_sinkage
    global RT_baseline_feature_sinkage
    global RB_baseline_feature_sinkage
    global sinkage_feature_filt_window
    global sinkage_feature_array
    exp_id = sub_msgs['sinkage_exp_msg'].description
    sinkage_features = ros2py(sub_msgs['sub_sinkage_features'].param_names, 
                              sub_msgs['sub_sinkage_features'].param_values)

    if exp_id == last_feature_sinkage_exp_id:
        LT_sinkage = sinkage_features['LT_mean_distance'] - LT_baseline_feature_sinkage
        LB_sinkage = sinkage_features['LB_mean_distance'] - LB_baseline_feature_sinkage
        MT_sinkage = sinkage_features['MT_mean_distance'] - MT_baseline_feature_sinkage
        MB_sinkage = sinkage_features['MB_mean_distance'] - MB_baseline_feature_sinkage
        RT_sinkage = sinkage_features['RT_mean_distance'] - RT_baseline_feature_sinkage
        RB_sinkage = sinkage_features['RB_mean_distance'] - RB_baseline_feature_sinkage
        #avg_sinkage, sinkage_feature_array = window_filter(np.mean, sinkage_feature_filt_window, 
        #                                                   LT_sinkage, sinkage_feature_array)
    else:
        LT_baseline_feature_sinkage = sinkage_features['LT_mean_distance']
        LT_sinkage = 0
        LB_baseline_feature_sinkage = sinkage_features['LB_mean_distance']
        LB_sinkage = 0
        MT_baseline_feature_sinkage = sinkage_features['MT_mean_distance']
        MT_sinkage = 0
        MB_baseline_feature_sinkage = sinkage_features['MB_mean_distance']
        MB_sinkage = 0
        RT_baseline_feature_sinkage = sinkage_features['RT_mean_distance']
        RT_sinkage = 0
        RB_baseline_feature_sinkage = sinkage_features['RB_mean_distance']
        RB_sinkage = 0
        #avg_sinkage = 0
    last_feature_sinkage_exp_id = exp_id 

    sinkage_msg = RiskMetric()
    sinkage_msg.name = "Sinkage Estimation"
    sinkage_msg.data_names = ['LT Sinkage', 'LB Sinkage',
                               'MT Sinkage', 'MB Sinkage',
                               'RT Sinkage', 'RB Sinkage']
    sinkage_msg.data_values = [LT_sinkage, LB_sinkage,
                                MT_sinkage, MB_sinkage,
                                RT_sinkage, RB_sinkage]
    pub_msgs['pub_sinkage'] = sinkage_msg


last_gt_sinkage_exp_id = None
baseline_gt_sinkage = 0
def process_sinkage_ground_truth(global_config, filter_config, pub_msgs, sub_msgs):
    global last_gt_sinkage_exp_id
    global baseline_gt_sinkage
    exp_id = sub_msgs['sinkage_exp_msg'].description
    testbed_sensor_states = ros2py(sub_msgs['sub_testbed_data'].param_names, 
                                   sub_msgs['sub_testbed_data'].param_values)
 
    if exp_id == last_gt_sinkage_exp_id:
        sinkage = testbed_sensor_states['CARRIAGE_V_POS_ACTUAL'] - baseline_gt_sinkage
    else:
        baseline_gt_sinkage = testbed_sensor_states['CARRIAGE_V_POS_ACTUAL']
        sinkage = 0
    last_gt_sinkage_exp_id = exp_id 

    sinkage_msg = RiskMetric()
    sinkage_msg.name = "Sinkage"
    sinkage_msg.data_names,\
    sinkage_msg.data_values = py2ros_nostr({'Ground Truth Sinkage': sinkage})
    pub_msgs['pub_gt_sinkage'] = sinkage_msg
