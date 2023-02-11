#!/usr/bin/env python

from planning_controls_modules.base_primitive import BasePrimitive

class BaseOperation(BasePrimitive):
    def __init__(self, name='base_operation',
                       root='operation',
                       verbose=True,
                       primary=False):
        super(BaseOperation, self).__init__(
            name,
            root,
            verbose
        )

        self.setup_operation(primary)
        self.create_configuration(name, root)

    def setup_operation(self, primary):
        self.primary        = primary
        self.filter_targets = True

        events = [ 'DISENGAGE', 'ENGAGE', 'STOP', 'START']

        effects = {
            'IDLE'         : {
                'DISENGAGE': self.disengage,
                'ENGAGE'   : self.engage,
                'STOP'     : self.stop,
                'START'    : self.engage_and_start,
            },
            'STANDING_BY'  : {
                'DISENGAGE': self.disengage,
                'ENGAGE'   : self.engage,
                'STOP'     : self.stop,
                'START'    : self.start,
            },
            'RUNNING'      : {
                'DISENGAGE': self.stop_and_disengage,
                'ENGAGE'   : self.stop_and_engage,
                'STOP'     : self.stop,
                'START'    : self.engage_and_start,
            },
        }

        transitions = {
            'IDLE'         : {
                'DISENGAGE': 'IDLE',
                'ENGAGE'   : 'STANDING_BY',
                'STOP'     : 'STANDING_BY',
                'START'    : 'RUNNING',
            },
            'STANDING_BY'  : {
                'DISENGAGE': 'IDLE',
                'ENGAGE'   : 'STANDING_BY',
                'STOP'     : 'STANDING_BY',
                'START'    : 'RUNNING',
            },
            'RUNNING'      : {
                'DISENGAGE': 'IDLE',
                'ENGAGE'   : 'STANDING_BY',
                'STOP'     : 'STANDING_BY',
                'START'    : 'RUNNING',
            },
        }

        self.setup_fsm(events, effects, transitions)

    def create_configuration(self, name, root):
        global_parameters = '/global'
        global_config = self.rospy.get_param(global_parameters)
        operation_config = self.rospy.get_param('/' + root + '/behaviors/' + name)

        self.configuration = {
            'operations' : operation_config,
            'rover': global_config['rovers'][global_config['rover']] 
        }

    def engage_and_start(self, command):
        self.engage(command)
        self.start(command)

    def stop_and_disengage(self, command):
        self.stop(command)
        self.disengage(command)

    def stop_and_engage(self, command):
        self.stop(command)
        self.engage(command)

    def stop_and_engage_and_start(self, command):
        self.stop(command)
        self.engage(command)
        self.start(command)

if __name__ == '__main__':
    operation = BaseOperation()

    operation.spin()
