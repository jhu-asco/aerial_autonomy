#!/usr/bin/env python
"""
Created on Tue Feb  7 01:21:27 2017

@author: gowtham
"""
import rospy
import rospkg
import os
from std_msgs.msg import String
from functools import partial
from python_qt_binding.QtCore import QObject, pyqtSignal


class RosEventTrigger(QObject):

    # Define signals
    quad_signal = pyqtSignal(str, name='quadStatus')
    arm_signal = pyqtSignal(str, name='armStatus')
    state_machine_signal = pyqtSignal(str, name='stateMachineStatus')

    def __init__(self, event_file_name):
        '''
        Class that loads an event file from a parameter
        It provides method to triggerEvents using ros publisher
        This class separates the GUI part of plugin from ros interface
        '''
        super(RosEventTrigger, self).__init__()
        # Create ros node
        if not event_file_name:
            # Get default event file name
            rospack = rospkg.RosPack()
            aerial_autonomy_path = rospack.get_path('aerial_autonomy')
            default_path = os.path.join(aerial_autonomy_path,
                                        'events/basic_events')
            if rospy.has_param("event_file"):
                print "Found event file: ", rospy.get_param('event_file')
            event_file_name = rospy.get_param('event_file', default_path)
            print "Event file name: ", event_file_name

        event_file = file(event_file_name, 'r')
        # Parse event file to save event list and event manager name
        event_line_list = [event_name.strip()
                           for event_name in event_file.read().splitlines()]
        event_file.close()
        # Define event manager name etc
        self.event_manager_name = event_line_list[0][:-1]
        self.event_names_list = event_line_list[1:]
        self.event_pub = rospy.Publisher('event_trigger',
                                         String, queue_size=1)
        # Define partial callbacks
        quadCallback = partial(self.statusCallback, signal=self.quad_signal)
        armCallback = partial(self.statusCallback, signal=self.arm_signal)
        stateMachineCallback = partial(self.statusCallback,
                                       signal=self.state_machine_signal)
        # Subscribers for quad arm and state machine updates
        rospy.Subscriber("quad_status", String, quadCallback)
        rospy.Subscriber("arm_status", String, armCallback)
        rospy.Subscriber("stat_machine_status", String, stateMachineCallback)

    def statusCallback(self, msg, signal):
        """
        Ros callback for status data from quad, arm, state machine
        """
        signal.emit(str(msg.data))

    def triggerEvent(self, event_name):
        """
        Function to publish ros event based on event name
        """
        if not rospy.is_shutdown():
            msg = String()
            msg.data = event_name
            self.event_pub.publish(msg)
