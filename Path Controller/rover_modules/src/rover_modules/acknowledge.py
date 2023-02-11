# Name: Samuel Chandler
# Email samxchandler@protoinnovations.com
#
# Description: 
#
# Class used for acknowledging commands and messages sent between ROS nodes
import rospy
import time
import copy
from rover_msgs.msg import Acknowledge

class acknowledge_sub:
    """
    Class used to subscribe to an acknowledge command and parse 
    acknowledges from a specific topic. The wait function allows 
    for waiting for a specific message (optional), with a specific 
    time (optional) and will return the message received on the 
    setup topic with these criteria.
    """
    def __init__(self, sub_topic, ack_topic, timeout=5.0, rate=.1):
        """
        Define default no ack received, timeout, and subscriber
        """
        self.wait_rate = rate
        self.ack_timeout = timeout
        self.rec_msg = Acknowledge() 
        self.rec_msg.topic = ack_topic
        self.reset_ack()
        
        self.sub_ack = rospy.Subscriber(sub_topic, Acknowledge, 
                                               self.handle_ack)
        self.wait_queue = []

    def wait(self, cmd_name, msg_time=None, msg_id=None):
        """
        Waits until timeout or acknowledge from topic is received then checks
        if the message is valid.
        returns true or data if an acknowledge is received, false otherwise
        """        
        start_time = time.time()
        count = 1;
        while not rospy.is_shutdown():
            current_time = time.time()
            if self.check_msg(cmd_name, msg_time, msg_id):
                self.valid_msg = True
            else:
                self.valid_msg = False
            self.wait_lock('wait')
            if ((self.rec_msg.status == "OK") and self.valid_msg):
                ret_msg = copy.copy(self.rec_msg)
                self.reset_ack()
                self.wait_unlock()
                return ret_msg

            elif (((self.rec_msg.status == "FAILED") and self.valid_msg) or 
                   (current_time - start_time) >= self.ack_timeout):
                self.reset_ack()
                self.wait_unlock()
                return 0
            else:
                self.wait_unlock()
                time.sleep(self.wait_rate)

    def handle_ack(self, ack_msg):
        """
        Receives acknowledge message over ros network, checks to see if
        the ack is from the expected topic. If so, sets the cmd name, 
        cmd time, topic and status
        """
        
        if self.rec_msg.topic == ack_msg.topic:
            self.wait_lock('handle_ack')
            self.rec_msg.cmd_name = ack_msg.cmd_name
            self.rec_msg.cmd_time = ack_msg.cmd_time
            self.rec_msg.cmd_id = ack_msg.cmd_id
            if ack_msg.data:
                self.rec_msg.data = ack_msg.data
            self.rec_msg.status = ack_msg.status
            self.wait_unlock()

    def check_msg(self, cmd_name, msg_time, msg_id):
        """
        check that the stored rec_msg is the same as the expected message 
        """        
        #Certain acknowledges can be validated from the command name and topic
        #command time and topic, or only the topic. Return 1 or these messages
        if (cmd_name != self.rec_msg.cmd_name):
            return 0

        if ((msg_id == None) and (msg_time==None)):
            return 1

        elif (msg_time == None):
            if (msg_id == self.rec_msg.cmd_id):
                return 1
            else:
                return 0
        elif (msg_id == None): 
            if ((msg_time.secs == self.rec_msg.cmd_time.secs) and
               (msg_time.nsecs == self.rec_msg.cmd_time.nsecs)):
                return 1
            else:
                return 0
        else:
            if ((msg_id == self.rec_msg.cmd_id) and 
               ((msg_time.secs == self.rec_msg.cmd_time.secs) and
               (msg_time.nsecs == self.rec_msg.cmd_time.nsecs))):
                return 1
            else:
                return 0

    def wait_lock(self, source):
        """
        Queue
        """
        if len(self.wait_queue) >=1:
            self.wait_queue.append(source)
            while self.wait_queue[0] != source:
                time.sleep(self.wait_rate)
        else:
            self.wait_queue.append(source)
    def wait_unlock(self):
        """
        Dequeue
        """
        #print self.wait_queue
        del self.wait_queue[0]
    
    def reset_ack(self):
        self.valid_msg = False
        self.rec_msg.status = "NO ACK"
        self.rec_msg.data = ""
        self.rec_msg.cmd_name = ""
        self.rec_msg.cmd_id = 0
        self.rec_msg.cmd_time.secs = 0
        self.rec_msg.cmd_time.nsecs = 0

class acknowledge_pub:
    """
    Class used to publish acknowledge messages with the setup topic.
    The publish method allows for setting the status and topic, and 
    optionally the msg, time and data. 
    """
    def __init__(self, topic):
        """
        Initialize publisher and message
        """
        self.pub_ack = rospy.Publisher(topic, Acknowledge, queue_size = 10)
        self.ack_msg = Acknowledge()

    def publish(self, cmd_name, status, topic, 
                msg_time=None, msg_id=None, data=None):
        """
        Publish message with status, topic etc.
        """
        self.ack_msg.cmd_name = cmd_name
        self.ack_msg.status = status
        self.ack_msg.topic = topic
    
        if msg_id:
            self.ack_msg.cmd_id = msg_id
        else:
            self.ack_msg.cmd_id = 0
        if msg_time:
            self.ack_msg.cmd_time = msg_time
        else:
            self.ack_msg.cmd_time.secs = 0
            self.ack_msg.cmd_time.nsecs = 0
        if data:
            self.ack_msg.data = data
        else:
            self.ack_msg.data = '' 
        self.pub_ack.publish(self.ack_msg)

class acknowledge_pub_sub:
    """
    Class used to integrate the acknowledge_pub and acknowledge_sub class.
    This is useful when forwarding acknowledges over certain topics. 

    Ex. - receiving publish a command, 
        - publishing that command,  
        - waiting for an acknowledge, 
        - receiving or not receiving that acknowledge,
        - forwarding the received/timed out acknowledge 
          to the original command sender
    """ 
    def __init__(self, ack_topic, wait_ack_topic, pub_ack_topic,
                 timeout=5.0, rate = .1):
        """
        initialize publisher and subscriber
        """
        self.pub_ack = acknowledge_pub(ack_topic)
        self.sub_ack = acknowledge_sub(ack_topic, wait_ack_topic, 
                                       timeout, rate)
        self.pub_ack_topic = pub_ack_topic

    def wait_pub(self, cmd_name, msg_time=None, msg_id=None): 
        ack = self.sub_ack.wait(cmd_name, msg_time, msg_id)
        if ack:
            if ack.data:
                self.pub_ack.publish(cmd_name,
                                     "OK", 
                                     self.pub_ack_topic, 
                                     msg_time,
                                     msg_id,  
                                     ack.data)
            else:
                self.pub_ack.publish(cmd_name,
                                     "OK", 
                                     self.pub_ack_topic, 
                                     msg_time,
                                     msg_id)
            return 1

        else:
            self.pub_ack.publish(cmd_name,
                                 "FAILED", 
                                 self.pub_ack_topic,
                                 msg_time,
                                 msg_id)
            return 0
