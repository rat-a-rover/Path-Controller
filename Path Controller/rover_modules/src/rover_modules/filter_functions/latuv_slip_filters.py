import numpy as np
from rover_modules.utils import ros2py
from rover_msgs.msg import StringArray

states = {'position': np.array([[]]), 
               'velocity': np.array([[]]), 
               'effort': np.array([[]])}

gps = {'x': np.array([]), 
            'y': np.array([]), 
            'z': np.array([])}

median_filter_size = 10

def filter_state(global_config, filter_config, pub_msgs, sub_msgs):
    if (('state_msg' not in sub_msgs) or 
       ('filtered_state_msg' not in pub_msgs)):
        return
    global states
    global median_filter_size
    rover_msg = sub_msgs['state_msg']
    filtered_msg = pub_msgs['filtered_state_msg']
    if states['position'].shape[0] < median_filter_size:
        states['position'] = (np.vstack((rover_msg.position, 
                                             states['position']))
                                  if states['position'].size 
                                  else np.array(rover_msg.position))
        states['velocity'] = (np.vstack((rover_msg.velocity, 
                                            states['velocity']))
                                  if states['velocity'].size 
                                  else np.array(rover_msg.velocity))
        states['effort'] = (np.vstack((rover_msg.effort, 
                                          states['effort']))
                                  if states['effort'].size 
                                  else np.array(rover_msg.effort))
    else:
        states['position'] = np.roll(states['position'], 1,
                                          axis=0)
        states['velocity'] = np.roll(states['velocity'], 1,
                                          axis=0)
        states['effort'] = np.roll(states['effort'], 1,
                                        axis=0)

        states['position'][0] = rover_msg.position
        states['velocity'][0] = rover_msg.velocity
        states['effort'][0] = rover_msg.effort
    filtered_msg.header = rover_msg.header
    filtered_msg.name = rover_msg.name
    filtered_msg.position = (np.median(states['position'], axis=0)
                             if len(states['position'].shape) > 1
                             else states['position'])
    filtered_msg.velocity = (np.median(states['velocity'], axis=0)
                             if len(states['velocity'].shape) > 1
                             else states['velocity'])
    filtered_msg.effort = (np.median(states['effort'], axis=0)
                           if len(states['effort'].shape) > 1
                           else states['effort'])

def filter_rtk_gps(global_config, filter_config, pub_msgs, sub_msgs):
    if ('rtk_gps_msg' not in sub_msgs):
        return
    global gps
    global median_filter_size
    gps_msg = sub_msgs['rtk_gps_msg']
    if gps['x'].size < median_filter_size:
        gps['x'] = np.append(gps_msg.twist.twist.linear.x, 
                                  gps['x'])
        gps['y'] = np.append(gps_msg.twist.twist.linear.y, 
                                  gps['y'])
        gps['z'] = np.append(gps_msg.twist.twist.linear.z, 
                                  gps['z'])
    else: 
        np.roll(gps['x'], 1)
        np.roll(gps['y'], 1)
        np.roll(gps['z'], 1)
        gps['x'][0] = gps_msg.twist.twist.linear.x
        gps['y'][0] = gps_msg.twist.twist.linear.y
        gps['z'][0] = gps_msg.twist.twist.linear.z

    filtered_gps_msg = pub_msgs['filtered_rtk_gps_msg']
    filtered_gps_msg.header = gps_msg.header
    filtered_gps_msg.child_frame_id = gps_msg.child_frame_id
    filtered_gps_msg.pose = gps_msg.pose
    filtered_gps_msg.twist.twist.linear.x = np.median(gps['x'])
    filtered_gps_msg.twist.twist.linear.y = np.median(gps['y'])
    filtered_gps_msg.twist.twist.linear.z = np.median(gps['z'])

def calc_slip(global_config, filter_config, pub_msgs, sub_msgs):
    if (('slip_msg' not in pub_msgs) or 
       ('filtered_state_msg' not in pub_msgs)):
        return
    filt_state_msg = pub_msgs['filtered_state_msg']
    filtered_vel = ros2py(filt_state_msg.name, filt_state_msg.velocity)
    if filtered_vel:
        filt_gps_msg = pub_msgs['filtered_rtk_gps_msg']  
        gps_linear_vel =  (filt_gps_msg.twist.twist.linear.x**2 
                           + filt_gps_msg.twist.twist.linear.y**2 
                           + filt_gps_msg.twist.twist.linear.z**2)**.5

        RR_linear_vel = (filtered_vel['RR/driving'] 
                         * global_config['measures']['wheel_radii']['LF']
                         if 'RR/driving' in filtered_vel else 0)
        LR_linear_vel = (filtered_vel['LR/driving'] 
                         * global_config['measures']['wheel_radii']['LR']
                         if 'LR/driving' in filtered_vel else 0)
        LF_linear_vel = (filtered_vel['LF/driving'] 
                         * global_config['measures']['wheel_radii']['RF']
                         if 'LF/driving' in filtered_vel else 0)
        RF_linear_vel = (filtered_vel['RF/driving'] 
                         * global_config['measures']['wheel_radii']['RR']
                         if 'RF/driving' in filtered_vel else 0)
        RR_slip = ((RR_linear_vel - gps_linear_vel) / RR_linear_vel
                  if RR_linear_vel > 0 else 0)
        LR_slip = ((LR_linear_vel - gps_linear_vel) / LR_linear_vel
                  if LR_linear_vel > 0 else 0)
        LF_slip = ((LF_linear_vel - gps_linear_vel) / LF_linear_vel
                  if LF_linear_vel > 0 else 0)
        RF_slip = ((RF_linear_vel - gps_linear_vel) / RF_linear_vel
                  if RF_linear_vel > 0 else 0)

        slip_msg = pub_msgs['slip_msg']
        slip_msg.name = 'slip'
        slip_msg.header = filt_state_msg.header
        slip_msg.sources = ['LF', 'RF', 'RR', 'LR']
        slip_msg.type = ['gps_calculated']

        slip_msg.param_names = [StringArray()
                                for i in xrange(len(slip_msg.sources))]
        slip_msg.param_values = [StringArray() 
                                 for i in xrange(len(slip_msg.sources))]
        slip_msg.param_names[0].data.append('LF Slip')
        slip_msg.param_names[1].data.append('RF Slip')
        slip_msg.param_names[2].data.append('LR Slip')
        slip_msg.param_names[3].data.append('RR Slip')
        slip_msg.param_values[0].data.append(str(LF_slip))
        slip_msg.param_values[1].data.append(str(RF_slip))
        slip_msg.param_values[2].data.append(str(LR_slip))
        slip_msg.param_values[3].data.append(str(RR_slip))
