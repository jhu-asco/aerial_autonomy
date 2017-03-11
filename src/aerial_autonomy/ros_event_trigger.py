#!/usr/bin/env python
"""
Created on Tue Feb  7 01:21:27 2017

@author: gowtham
"""
import rospy
import rospkg
import os
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from functools import partial
from python_qt_binding.QtCore import QObject, pyqtSignal


def parse_event_file(event_folder, event_file_name, event_name_list):
    """
    Parse an event file to store all the event names. If an event
    file includes another event file, it recursively crawls through
    all event files and saves event names
    """
    event_file_path = os.path.join(event_folder, event_file_name)
    event_file = file(event_file_path, 'r')
    # Parse event file to save event list
    event_line_list = [event_name.strip()
                       for event_name in event_file.read().splitlines()]
    event_file.close()
    for event_name in event_line_list[1:]:
        event_name_split = event_name.split('/')
        if len(event_name_split) == 2:
            parse_event_file(
                event_folder,
                event_name_split[0],
                event_name_list)
        else:
            event_name_list.append(event_name)


class RosEventTrigger(QObject):
    """
    Trigger events based on event name. Create event name list
    based on event file
    """
    ## Send quad sensor/state data as string
    status_signal = pyqtSignal(str, name='systemStatus')
    ## Send pose command received from Rviz
    pose_command_signal = pyqtSignal(PoseStamped, name='poseCommand')

    def __init__(self, event_file_path):
        '''
        Class that loads an event file from a parameter
        It provides method to triggerEvents using ros publisher
        This class separates the GUI part of plugin from ros interface
        '''
        super(RosEventTrigger, self).__init__()
        # Create ros node
        if not event_file_path:
            # Get default event file name
            rospack = rospkg.RosPack()
            aerial_autonomy_path = rospack.get_path('aerial_autonomy')
            default_path = os.path.join(aerial_autonomy_path,
                                        'events/basic_events')
            if rospy.has_param("event_file"):
                print "Found event file: ", rospy.get_param('event_file')
            event_file_path = rospy.get_param('event_file', default_path)
            print "Event file name: ", event_file_path

        event_file = file(event_file_path, 'r')
        # Parse event file to save event list and event manager name
        event_line_list = [event_name.strip()
                           for event_name in event_file.read().splitlines()]
        event_file.close()
        ## Define event manager name
        self.event_manager_name = event_line_list[0][:-1]

        ## Event names used to create buttons
        self.event_names_list = []

        event_folder, event_file_name = event_file_path.rsplit('/', 1)
        parse_event_file(event_folder, event_file_name, self.event_names_list)

        ## Ros publisher to trigger events based on name
        self.event_pub = rospy.Publisher('event_trigger',
                                         String, queue_size=1)
        ## Ros publisher to trigger pose event
        self.pose_command_pub = rospy.Publisher('pose_command_combined',
                                                PoseStamped, queue_size=1)
        # Define partial callbacks
        statusCallback = partial(self.statusCallback, signal=self.status_signal)
        poseCommandCallback = lambda pose_command : self.pose_command_signal.emit(pose_command)
        # Subscribers for quad arm and state machine updates
        rospy.Subscriber("system_status", String, statusCallback)

        # Subscribe to position commands (from Rviz)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, poseCommandCallback) 

    def statusCallback(self, msg, signal):
        """
        Ros callback for status data from quad, arm, state machine
        """
        signal.emit(str(msg.data))

    def triggerPoseCommand(self, pose):
        """
        Publish pose command event for state machine
        """
        if not rospy.is_shutdown():
            self.pose_command_pub.publish(pose) 

    def triggerEvent(self, event_name):
        """
        Function to publish ros event based on event name
        """
        if not rospy.is_shutdown():
            msg = String()
            msg.data = event_name
            self.event_pub.publish(msg)
