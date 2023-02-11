def extract_status(global_config, filter_config, pub_msgs, sub_msgs):
    if 'status_msg' not in sub_msgs:
        return
    msg = sub_msgs['status_msg']
    if not msg.param_names:
        return
    data_names = msg.param_names[0].data
    motor_names = pub_msgs.keys()
    motor_types = dict(zip(msg.sources, msg.type))
    msg_data = dict(zip(msg.sources, msg.param_values))
    for motor_name, motor_data in msg_data.items():
        if motor_name not in motor_names:
            continue
        motor_type = motor_types[motor_name]
        pub_msgs[motor_name].name = motor_name
        pub_msgs[motor_name].type = motor_type
        for idx, data in enumerate(motor_data.data):
            data_name = data_names[idx]
            setattr(pub_msgs[motor_name], data_name, data)
