#!/usr/bin/env python
from collections import OrderedDict

def ros2py(param_keys, param_values):
    return dict(zip(param_keys, param_values))

def py2ros(params):
    ordered_params = OrderedDict(params)
    return ([k for k in ordered_params.keys()], 
            [str(v) for v in ordered_params.values()])

def py2ros_nostr(params):
    ordered_params = OrderedDict(params)
    return ([k for k in ordered_params.keys()], 
            [v for v in ordered_params.values()])
