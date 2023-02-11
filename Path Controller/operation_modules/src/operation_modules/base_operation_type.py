#!/usr/bin/env python

from planning_controls_modules.base_primitive_type import BasePrimitiveType

import time

class BaseOperationType(BasePrimitiveType):
    def __init__(self, name='base_operation_type',
                       configuration=None,
                       publish=None,
                       verbose=False,
                       mobility=True):
        super(BaseOperationType, self).__init__(
            name,
            configuration,
            publish,
            verbose
        )
        self.mobility = mobility

    def setup(self, state, data):
        if data['command']['event'] in ['ENGAGE']:

            state['statics']  = self.default('statics', 'START')
            state['statics']  = self.physics('statics', state['statics'], data)

            self.publish(data['command'], state['statics'])

        elif data['command']['event'] in ['DISENGAGE']:

            state['statics']  = self.default('statics', 'STOP')

            self.publish(data['command'], state['statics'])

        return state

    def runtime(self, state, data):
        state.setdefault('statics', dict())
        state.setdefault('dynamics', self.default('dynamics', 'START'))

        if data['command']['event'] in ['START']:

            state['dynamics'] = self.physics('dynamics', state['dynamics'], data)

            self.publish(data['command'], state['dynamics'])

        elif data['command']['event'] in ['STOP']:

            self.stop()

            state['dynamics'] = self.default('dynamics', 'STOP')

            self.publish(data['command'], state['dynamics'])

        return state

    def default(self, stage, event):
        commands = dict()

        target_group = self.configuration['rover']['wheels'] \
                       if self.mobility else self.configuration['rover']['manipulators']
        for group_id in target_group:
            if stage in self.__DIMENSIONS__:
                for dimension in self.__DIMENSIONS__[stage]:
                    commands[group_id + '/' + dimension] = \
                      dict({ 'event': event,
                             'name': self.__NAMES__[stage][dimension],
                             'targets': list([group_id + '/' + dimension]),
                             'type': self.__TYPES__[stage][dimension],
                             'parameters': { self.__PARAM_NAMES__[stage][dimension]: '0.0' } })

        return commands

    def physics(self, stage, commands, data):
        if data['command']['parameters'] is None:
            return commands

        for parameter in self.__PARAMETERS__:
            if parameter not in data['command']['parameters']:
                return commands

        return getattr(self, stage)(commands, data)

    def statics(self, commands, data):
        raise NotImplementedError("Implement your 'statics' method!")
        return commands

    def dynamics(self, commands, data):
        raise NotImplementedError("Implement your 'dynamics' method!")
        return commands
