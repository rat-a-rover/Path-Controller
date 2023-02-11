import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

def fix_gps_frame(global_config, filter_config, pub_msgs, sub_msgs):
    input_msg =  sub_msgs['gps_msg']
    output_msg = NavSatFix()
    output_msg.header.frame_id = 'GPS'
    output_msg.header.stamp = input_msg.header.stamp
    output_msg.status = input_msg.status
    output_msg.latitude = input_msg.latitude
    output_msg.longitude = input_msg.longitude
    output_msg.altitude = input_msg.altitude
    output_msg.position_covariance = input_msg.position_covariance
    output_msg.position_covariance_type = input_msg.position_covariance_type
    pub_msgs['fix_gps_msg'] = output_msg

def fix_imu_frame(global_config, filter_config, pub_msgs, sub_msgs):
    input_msg =  sub_msgs['imu_msg']
    output_msg = Imu()
    output_msg.header.frame_id = 'IMU'
    output_msg.header.stamp = input_msg.header.stamp
    output_msg.orientation = input_msg.orientation
    output_msg.angular_velocity = input_msg.angular_velocity
    output_msg.linear_acceleration = input_msg.linear_acceleration
    quat = np.array([output_msg.orientation.x, 
                     output_msg.orientation.y, 
                     output_msg.orientation.z, 
                     output_msg.orientation.w])
    # Only publish valid IMU data
    if round(np.linalg.norm(quat)) == 1:
        pub_msgs['fix_imu_msg'] = output_msg
    else:
        del pub_msgs['fix_imu_msg']
