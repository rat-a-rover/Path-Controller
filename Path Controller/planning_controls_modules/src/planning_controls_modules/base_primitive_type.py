#!/usr/bin/env python
#
# ============================================================================
#
#       Filename:  base_primitive_type.py
#
#    Description:  Base class for primitive types
#
#        Version:  1.0
#        Created:  12/01/2018
#       Revision:  none
#
#         Author:  JP Vega
#     Maintainer:  Samuel Chandler
#          Email:  samxchandler@protoinnovations.com
#   Organization:  ProtoInnovations
#
# ============================================================================
#

import rospy

class BasePrimitiveType(object):
    def __init__(self, name='base_primitive_type',
                       configuration=None,
                       publish=None,
                       verbose=False):
        self.name          = name
        self.configuration = configuration
        self.publish       = publish
        self.verbose       = verbose

        self.pit = None
        self.isr = None
        self.seq = None
        self.ctr = None

        self.update = {
            'setup'   : self.setup,
            'runtime' : self.runtime,
        }

    def start(self, rate, data, commands, sequence, counter):
        if rate  > 0:
            if self.pit is not None:
                self.pit.shutdown()

            self.seq = sequence
            self.ctr = counter

            def wrapped_update(event):
                if self.seq is None or self.ctr is None:
                    return

                idx   = self.ctr % len(self.seq)
                value = self.seq[idx]

                self.isr and self.isr(event, data, commands, value)

                self.ctr = self.ctr + 1

            self.pit = rospy.Timer(rospy.Duration(1./rate), wrapped_update)

    def stop(self):
        self.pit and self.pit.shutdown()
        self.pit = None

    def noop(self, state, data):
        return state

    def setup(self, state, data):
        return state

    def runtime(self, state, data):
        return state

    def is_running(self):
        return True if self.pit else False
