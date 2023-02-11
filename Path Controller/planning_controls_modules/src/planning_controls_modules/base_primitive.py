#!/usr/bin/env python

import rospy
import threading
import copy

from rover_msgs.msg import Command, Control, Event, Config
from fsm import FSM
from rover_modules.utils import ros2py, py2ros


class BasePrimitive(object):
    def __init__(self, name='base_primitive', root='base_root', verbose=False):
        self.name = name
        self.root = root
        self.verbose = verbose

        self.rospy = rospy

        self.setup_ros()
        self.setup_base_primitive()

    def setup_ros(self):
        self.id  = 0

        self.rospy.init_node(self.name, anonymous=True)

        self.rospy.Subscriber(
            'orchestrator/command',
            Command,
            self.orchestrator_command
        )

        self.rospy.Subscriber(
            '/mission/multiplexer/switch',
            Config,
            self.configure_source
        )

        self.primitive_command_publisher = rospy.Publisher(
            self.root+'/'+self.name+'/command',
            Command,
            queue_size=10
        )

        self.primitive_event_publisher = rospy.Publisher(
            '/'.join(['', 'global', self.root, self.name, 'event']),
            Event,
            queue_size=10
        )

        print('<<<', '/'.join(['', 'global', self.root, self.name, 'event']), '>>>')

    def setup_base_primitive(self):
        # state = type[type].update[stage[uuid]](state[uuid], data)
        # type[type] : primitive type  [str]  ==> primitive type [object]
        # stage[uuid]   : primitive uuid  [hash] ==> primitive stage   [str]
        # update[stage] : primitive stage [str]  ==> primitive update  [method]
        # state[uuid]   : primitive uuid  [hash] ==> primitive state   [dict]
        self.types    = dict()
        self.stage    = dict()
        self.state    = dict()
        self.data     = dict()
        self.forced_delay = 1.0

    def setup_fsm(self, events, effects, transitions):
        self.FSM = FSM(events, effects, transitions, self.error)

    def configure_source(self, switch_msg):
        params = ros2py(switch_msg.param_names, switch_msg.param_values)
        override_delay = 0.001
        sequenced_delay = 1.0
        if 'joystick' in params:
            self.forced_delay = override_delay if (params['joystick'] == 'enable') else sequenced_delay
        elif 'terminal' in params:
            self.forced_delay = sequenced_delay if (params['terminal'] == 'enable') else override_delay
        else:
            self.forced_delay = sequenced_delay

    def orchestrator_command(self, ros_command):
        if ros_command.name != self.name:
            return

        if self.verbose:
            self.rospy.loginfo(
                self.name[:10].ljust(10)        + ' <<   '  + \
                ros_command.event               + ' '       + \
                ros_command.name                + ' '       + \
                str(ros_command.targets)        + ' '       + \
                str(ros_command.param_values)
            )

        self.handle_command(
            ros_command.event,
            {
                'timestamp' : ros_command.timestamp,
                'id'        : ros_command.id,
                'start'     : ros_command.start,
                'timeout'   : ros_command.timeout,
                'event'     : ros_command.event,
                'name'      : ros_command.name,
                'targets'   : ros_command.targets,
                'type'      : ros_command.type,
                'parameters': ros2py(
                    ros_command.param_names,
                    ros_command.param_values
                )
            }
        )

    def handle_command(self, event, command):
        if self.FSM is not None:
            self.FSM.event(event, command)

    def primitive_command(self, command, new_commands):
        self.id = self.id + 1
        ros_commands = []
        ## JANK CODE ##
        blade_detected = False
        BLADE_DELAY = 21
        ##############
        for idx, ncommand in enumerate(new_commands.values()):
            new_command = copy.deepcopy(ncommand) 

            if self.filter_targets:
                new_command['targets'] = [
                    target
                    for target in new_command['targets']
                    if '/'.join(target.split('/')[:-1]) in command['targets']
                ]

                if len(new_command['targets']) == 0:
                    continue

            param_names, param_values = py2ros(new_command['parameters'])

            # where the blade delay occurs
            if new_command['name'] == 'sweep' and (new_command['type'] == 'simple_blade'):
                blade_detected = True
                blade_cmd_idx = idx

            ros_commands.append(Command(
                timestamp=rospy.get_rostime(),
                id=self.id,
                start=command['start'],
                timeout=command['timeout'],
                name=new_command['name'],
                event=new_command['event'],
                targets=new_command['targets'],
                type=new_command['type'],
                param_names=param_names,
                param_values= param_values
            ))

        if blade_detected:
            cmd = ros_commands[blade_cmd_idx]
            if self.verbose:
                self.rospy.loginfo(
                    self.name[:10].ljust(10) + '   >> '  + \
                    cmd.event                + ' '       + \
                    cmd.name                 + ' '       + \
                    str(cmd.targets)         + ' '       + \
                    str(cmd.param_values
                ))
            self.primitive_command_publisher.publish(cmd)
            print('send blade command: ', cmd)
            rospy.sleep(BLADE_DELAY)

        for idx, cmd in enumerate(ros_commands):
            if self.verbose:
                self.rospy.loginfo(
                    self.name[:10].ljust(10) + '   >> '  + \
                    cmd.event                + ' '       + \
                    cmd.name                 + ' '       + \
                    str(cmd.targets)         + ' '       + \
                    str(cmd.param_values
                ))
            if not blade_detected or idx != blade_cmd_idx:
                self.primitive_command_publisher.publish(cmd)

    def update(self):
        if 'command' in self.data:
            xtype  = self.data['command']['type']
            xstage = self.stage

            if xtype in self.types:
                self.state = self.types[xtype].update[xstage](
                    self.state,
                    self.data
                )
            else:
                raise(
                    NotImplementedError(self.name,'of type',xtype,'not supported')
                )

    def engage(self, command):
        if self.primary is True:
            self.handle_event(command, 'ENGAGE', 'setup')
            rospy.sleep(self.forced_delay)

        self.publish_event(command)

    def start(self, command):
        self.handle_event(command, 'START', 'runtime')
        self.publish_event(command)

    def stop(self, command):
        self.handle_event(command, 'STOP', 'runtime')
        self.publish_event(command)

    def disengage(self, command):
        if self.primary is True:
            rospy.sleep(self.forced_delay)
            self.handle_event(command, 'DISENGAGE', 'setup')

        self.publish_event(command)

    def handle_event(self, command, event, stage):
        '''
        Set stage (runtime or setup) and data (command)
        then update primitive types
        '''
        command = dict(command)
        command['event'] = event
        self.data['command'] = command
        self.stage = stage
        self.update()

    def publish_event(self, command):

        ros_event = Event(
            timestamp=self.rospy.get_rostime(),
            id=self.id,
            name=command['event'],
            source=command['name'],
            param_names=['targets', 'id'],
            param_values=[str(command['targets']), str(command['id'])]
        )

        if self.verbose:
            self.rospy.loginfo(
                self.name[:10].ljust(10)    + '    # '  + \
                ros_event.name              + ' '       + \
                str(ros_event.source)       + ' '       + \
                str(ros_event.param_values)
            )

        self.primitive_event_publisher.publish(ros_event)

    def error(self, type, info):
        self.rospy.logerr(type + str(info))

    def is_running(self):
        if 'command' in self.data:
            xtype = self.data['command']['type']
            return self.types[xtype].is_running()
        else:
            return False

    def spin(self, *args, **kwargs):
        self.rospy.spin()
