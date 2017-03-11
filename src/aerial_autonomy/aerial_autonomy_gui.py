#!/usr/bin/env python
"""
Generate a GUI to trigger events for state machine

@author: gowtham
"""
import argparse
import sip
from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import (QLabel, QVBoxLayout,
                                         QGridLayout, QWidget,
                                         QTextEdit, QPushButton,
                                         QSlider)
from python_qt_binding.QtCore import *
from ros_event_trigger import RosEventTrigger
from argparse import ArgumentParser
from functools import partial

# %%


class EventTransmissionGUI(Plugin):
    """
    GUI to send events from User to logic state machine
    """
    def __init__(self, context):
        """
        Create Qt GUI using the event file
        """
        super(EventTransmissionGUI, self).__init__(context)
        self.setObjectName('ManualEventTriggerGUI')
        parser = ArgumentParser()

        # Add argument(s) to the parser.
        args = self._parse_args(context.argv())

        ## Create Event trigger
        self.event_trigger = RosEventTrigger(args.event_file)

        ## Parent container to store buttons, textboxes
        self._container = QWidget()
        # Set title of the parent container window
        self._container.setWindowTitle(self.event_trigger.event_manager_name)
        ## layout for the parent container
        self._layout = QVBoxLayout()
        self._container.setLayout(self._layout)

        # Create Textboxes and add to Layout
        self._layout.addWidget(QLabel('State Machine State'))
        ## Textbox to show sytem status
        self.system_status_textbox = QTextEdit()
        self.system_status_textbox.setReadOnly(True)
        self._layout.addWidget(self.system_status_textbox)
        self._layout.addWidget(QLabel('System Status'))

        # Create height slider
        self._layout.addWidget(QLabel('Pose Command Height (m)'))
        ## Height slider to adjust z coordinate for pose command
        ## \todo Matt: Load slider settings from param file
        self.height_slider = QSlider(Qt.Horizontal)
        self.height_slider.setMinimum(1.)
        self.height_slider.setMaximum(20)
        self.height_slider.setValue(2)
        self.height_slider.setTickPosition(QSlider.TicksBelow)
        self.height_slider.setTickInterval(1)
        self._layout.addWidget(self.height_slider)
        # Add button for triggering pose command
        ## Container for pose event related objects: slide etc
        ## \todo Matt: Reset slider value based on current quad height
        self.pose_command_container = QWidget()
        ## Pose command layout
        self.pose_command_layout = QGridLayout()
        self.pose_command_container.setLayout(self.pose_command_layout)
        ## x pose label to display position command from rviz to user
        self.pose_x = QLabel('x: -')
        ## y pose label to display position command from rviz to user
        self.pose_y = QLabel('y: -')
        ## z pose label to display position command from rviz to user
        self.pose_z = QLabel("z: {0:.2f}".format(self.height_slider.value()))
        self.height_slider.valueChanged.connect(self.updateHeight)
        self.pose_command_layout.addWidget(self.pose_x, 0, 0)
        self.pose_command_layout.addWidget(self.pose_y, 0, 1)
        self.pose_command_layout.addWidget(self.pose_z, 0, 2)
        ## Button to send the pose command to state machine as poseyaw event
        self.send_pose_command_button = QPushButton("Send Pose Command")
        self.send_pose_command_button.clicked.connect(self.poseCommandButtonCallback)
        self.pose_command_layout.addWidget(self.send_pose_command_button, 0, 3)
        self._layout.addWidget(self.pose_command_container)
        ## Pose command container to store pose from Rviz and send to state machine
        self.pose_command = None

        # Define and connect buttons
        self._layout.addWidget(QLabel('Event Triggers'))
        ## Continer to store event triggering buttons
        self.button_container = QWidget()
        ## List of push buttons to trigger events
        self.push_buttons = list()
        ## Layout for the push buttons
        self.button_layout = QGridLayout()
        self.button_container.setLayout(self.button_layout)
        button_index = 0
        for event_name in self.event_trigger.event_names_list:
            self.push_buttons.append(QPushButton(event_name))
            partial_fun = partial(self.event_trigger.triggerEvent,
                                  event_name=event_name)
            self.push_buttons[-1].clicked.connect(partial_fun)
            row, col = self.get_row_col(button_index, args.grid_cols)
            self.button_layout.addWidget(self.push_buttons[-1], row, col)
            button_index += 1
        self._layout.addWidget(self.button_container)

        context.add_widget(self._container)

        # Add textboxes to update hooks from eventTrigger class
        # Define Partial callbacks
        systemStatusCallback = partial(
            self.updateStatus, text_box=self.system_status_textbox)
        # Connect Event Triggers
        self.event_trigger.status_signal.connect(
            systemStatusCallback)
        self.event_trigger.pose_command_signal.connect(self.poseCommandCallback)

    def _parse_args(self, argv):
        """
        Parse extra arguments when plugin is deployed in standalone mode
        """
        parser = argparse.ArgumentParser(
            prog='aerial_autonomy', add_help=False)
        EventTransmissionGUI.add_arguments(parser)
        return parser.parse_args(argv)

    @staticmethod
    def add_arguments(parser):
        """
        Notify rqt_gui that this plugin can parse these extra arguments
        """
        group = parser.add_argument_group(
            'Options for aerial autonomy gui plugin')
        group.add_argument("-e", "--event_file", type=str,
                           default='', help="Event file")
        group.add_argument("-c", "--grid_cols", type=int,
                           default=3, help="Number of columns in grid")

    def get_row_col(self, button_index, ncols):
        """
        Automatically find the row and col to add the button
        to in a grid based on index of the button
        """
        col_index = button_index % ncols
        row_index = int((button_index - col_index) / ncols)
        return(row_index, col_index)

    def poseCommandCallback(self, pose):
        """
        Saves pose command and updates command display
        """
        self.pose_command = pose
        self.pose_x.setText("x: {0:.2f}".format(self.pose_command.pose.position.x))
        self.pose_y.setText("y: {0:.2f}".format(self.pose_command.pose.position.y))

    def poseCommandButtonCallback(self):
        """
        Publishes stored pose command after setting height from slider
        """
        if self.pose_command:
            self.pose_command.pose.position.z = self.height_slider.value()
            self.event_trigger.triggerPoseCommand(self.pose_command)
            # Reset pose command to avoid accidental triggering
            self.pose_command = None
            self.pose_x.setText('x: -')
            self.pose_y.setText('y: -')
        else:
            print "No pose command to trigger"

    def updateHeight(self):
        """
        Updates height label based on slider value
        """
        self.pose_z.setText("z: {0:.2f}".format(self.height_slider.value()))

    def updateStatus(self, status, text_box):
        """
        Generic placeholder function to update text box
        """
        if not sip.isdeleted(text_box):
            text_box.setText(status)
