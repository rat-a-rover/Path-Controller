# Name: Samuel Chandler
# Email: samxchandler@protoinnovations.com

# Buffer used to manage and synchronize dictionary data

import threading
import copy
import sys


class Synchronizer(object):
    def __init__(self, levels):
        """
        Initialize the buffer, trash and thread lock. Buffer levels (the size
        of the data being pushed, popped, and synced) has a min size of 2.
        Smaller values do not represent valid dictionaries
        """
        if levels < 2:
            sys.exit('Synchronizer: invalid number of buffer levels')
        self.buffer = {'levels': levels,  # number of data args for push/pop
                       'data': dict()}
        self.trash = list()
        self.lock = threading.Lock()

    def push(self, data):
        """
        Given 'data' list, Push 'data' into the buffer. Calling 'sync' does 
        not remove pushed 'data'. 

        Returns nothing
        """
        self.lock.acquire()
        if len(data) == (self.buffer['levels']):
            self.__set_buffer_defaults(self.buffer['data'], data)[data[-2]] = \
                data[-1]
        self.lock.release()

    def pop(self, data):
        """
        Given 'data' list, push the 'data' into the buffer. Calling the sync 
        method removes all popped 'data'. 

        Returns nothing
        """
        self.lock.acquire()
        if len(data) == (self.buffer['levels']):
            self.__set_buffer_defaults(self.buffer['data'], data)[data[-2]] = \
                data[-1]
            self.trash.append(data[0:-1])
        self.lock.release()

    def sync(self):
        """
        Flushes and returns current buffer. 

        Returns nothing.
        """
        self.lock.acquire()
        sbuffer = copy.deepcopy(self.buffer['data'])
        self.__flush()
        self.lock.release()
        return sbuffer

    def check_buffer_size(self, keys):
        """
        Takes a list of keys and 

        Returns length of buffer at buffer[key[0]][key[1]...[key[n]]
        """
        self.lock.acquire()
        if len(keys) < self.buffer['levels']:
            size = self.__calc_buffer_size(self.buffer['data'], keys)
        else:
            size = 0
        self.lock.release()
        return size

    def __calc_buffer_size(self, buffer, keys):
        """
        Traverses a dictionary and returns the length
        """
        try:
            if (len(keys) > 1) and type(buffer[keys[0]] is dict):
                return self.__calc_buffer_size(buffer[keys[0]], keys[1:])
            elif type(buffer[keys[0]]) is dict:
                return len(buffer[keys[0]]) 
            else:
                return 1
        except KeyError:
            return 0

    def __set_buffer_defaults(self, buffer, data):
        """
        Given a 'buffer' dictionary and 'data' list, recursively set 'buffer' 
        defaults with 'data'

        Returns a buffer with defaults
        """
        if len(data) > 2:
            return self.__set_buffer_defaults(
                buffer.setdefault(data[0], dict()), data[1:])
        else:
            return buffer

    def __del_buffer_data(self, buffer, data):
        """
        Given a 'buffer' dictionary and 'data' list, recursively delete 
        elements in 'buffer' that matches 'data'

        Returns True if an element is delete, false otherwise
        """
        if data[0] in buffer:
            if len(data) > 1:
                if ((self.__del_buffer_data(buffer[data[0]], data[1:])) and
                    (not buffer[data[0]])):
                    del buffer[data[0]]
                    return True
                else:
                    return False
            else:
                del buffer[data[0]]
                return True
        else:
            return False

    def __flush(self):
        """ 
        Deletes common elements in 'trash' and 'buffer'. 

        Returns nothing.
        """
        strash = copy.deepcopy(self.trash)
        self.trash = list()
        for data in strash:
            self.__del_buffer_data(self.buffer['data'], data)
