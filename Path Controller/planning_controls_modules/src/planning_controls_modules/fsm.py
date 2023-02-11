#!/usr/bin/env python

from collections import deque as Queue

'''
TO DO
- Handle asynchronous queue and concurrency
'''

class FSM(object):
    def __init__(self, events, effects, transitions, error=None):
        self.states = [ 'IDLE', 'STANDING_BY', 'RUNNING' ]

        self.events = events or [ 'DISENGAGE', 'ENGAGE', 'STOP', 'START']

        self.effects = effects or {
            'IDLE'        : { event : None for event in self.events },
            'STANDING_BY' : { event : None for event in self.events },
            'RUNNING'     : { event : None for event in self.events },
        }

        self.transitions = transitions or {
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

        self.error = error or self.default_error

        self.event_queue = Queue() # Not used yet

        self.reset()

    def reset(self):
        self.event_queue.clear()
        self.state = 'IDLE'

    def default_error(self):
        pass

    def event(self, event, data=None):
        if event not in self.events:
            self.error('EVENT_ERROR', (self.state, event, data))
            return

        if self.state in self.effects and event in self.effects[self.state]:
            self.effects[self.state][event](data)

        self.state = self.transitions[self.state][event]

        '''try:
            error = self.effects[self.state][event](data)

        except NotImplementedError as e:
            self.reset()
            self.error('ACTION_ERROR', (self.state, event, data))

        except Exception as e:
            self.error('ACTION_ERROR', (self.state, event, data))
            raise(e)

        if error is None:
            try:
                self.state = self.transitions[self.state][event]

            except NotImplementedError as e:
                self.reset()
                self.error('TRANSITION_ERROR', (self.state, event, data))

            except Exception as e:
                self.error('TRANSITION_ERROR', (self.state, event, data))
                raise(e)
        '''
