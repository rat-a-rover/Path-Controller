import glob
from os.path import dirname, join

import rospy
# Rover Battery Sensor and Emergency Codes
from rover_msgs.msg import *

#import all standard message types
from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *

from importlib import import_module


class MsgFilters:
    MODULES_DIR = 'rover_modules'
    FILTER_DIR = 'filter_functions'
    def __init__(self, config, rover):
        self.rover = rover
        self.init_configs(config)
        self.init_filters()

    def init_configs(self, config):
        self.global_config = config['rovers'][self.rover]
        self.filter_config = config['filter_processor']
    
    def init_filters(self):
        # get full directory list of modules
        modules = glob.glob(join(dirname(__file__), 
                                 self.FILTER_DIR +'/*.py'))
        function_names = self.filter_config['functions'].values()
        self.functions = {}
        for module_path in modules:
            name = module_path.split('/')[-1].split('.py')[0]
            if name != '__init__':
                module = import_module(self.MODULES_DIR + '.' 
                                       + self.FILTER_DIR + '.' + name)
                for f_name in function_names:
                    if hasattr(module, f_name):
                        #TODO If a function of the same name exists in
                        # multiple modules then only one of them will be
                        # used. Need to be more explicit with importing
                        # but somehow perserve processing simplicity
                        # Maybe add path arg to import module?
                        # dictionary of filter_name: filter_function 
                        # filter_function defined in configuration files
                        self.functions[f_name] = getattr(module, f_name)

    def process_msgs(self, sub_msgs):
        pub_msgs = {topic_dict['msg']['name']:
                        eval(topic_dict['msg']['type'] + '()')
                        for topic, topic_dict in
                        self.filter_config['publish_topics'].items()}

        for function in sorted(self.filter_config['functions'].keys()):
            
            self.functions[self.filter_config['functions'][function]]\
                (self.global_config, self.filter_config, pub_msgs, sub_msgs)
 
        return pub_msgs
