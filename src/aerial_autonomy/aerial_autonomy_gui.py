#!/usr/bin/env python
"""
@package aerial_autonomy GUI source code
to provide user the ability to
trigger events from GUI to state machine

@package aerial_autonomy.aerial_autonomy_gui
Generate a GUI to trigger events for state machine

@author: gowtham
"""
import argparse
import numpy as np
import sip
from aerial_autonomy.msg import VelocityYaw
from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import (QLabel, QVBoxLayout,
                                         QGridLayout, QWidget,
                                         QTextEdit, QPushButton,
                                         QSlider, QTabWidget)
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

        # Create Event trigger
        self.event_trigger = RosEventTrigger(args.event_file)

        # Parent container to store buttons, textboxes
        self._container = QTabWidget()
        self._status_tab = QWidget()
        self._additional_commands_tab = QWidget()
        # Set title of the parent container window
        self._status_tab.setWindowTitle(self.event_trigger.event_manager_name)
        # layout for the parent container
        self._layout = QVBoxLayout()
        self._status_tab.setLayout(self._layout)

        # Create Textboxes and add to Layout
        self._layout.addWidget(QLabel('State Machine State'))
        # Textbox to show sytem status
        self.system_status_textbox = QTextEdit()
        self.system_status_textbox.setReadOnly(True)
        self._layout.addWidget(self.system_status_textbox)

        # Define and connect buttons
        self._layout.addWidget(QLabel('Event Triggers'))
        # Continer to store event triggering buttons
        self.button_container = QWidget()
        # List of push buttons to trigger events
        self.push_buttons = list()
        # Layout for the push buttons
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

        # Create additional commands
        self._additional_commands_layout = QVBoxLayout()
        self._additional_commands_tab.setLayout(
            self._additional_commands_layout)
        # Create height slider
        self._additional_commands_layout.addWidget(
            QLabel('Pose Command Height (m)'))
        # Height slider to adjust z coordinate for pose command
        # \todo Matt: Load slider settings from param file
        self.height_slider = self.createSlider(1.0, 20.0, 2.0, 1.0)
        self._additional_commands_layout.addWidget(self.height_slider)
        # Add button for triggering pose command
        # Container for pose event related objects: slide etc
        # \todo Matt: Reset slider value based on current quad height
        self.pose_command_container = QWidget()
        # Pose command layout
        self.pose_command_layout = QGridLayout()
        self.pose_command_container.setLayout(self.pose_command_layout)
        # x pose label to display position command from rviz to user
        self.pose_x = QLabel('x: -')
        # y pose label to display position command from rviz to user
        self.pose_y = QLabel('y: -')
        # z pose label to display position command from rviz to user
        self.pose_z = QLabel("z: {0:.2f}".format(self.height_slider.value()))
        self.height_slider.valueChanged.connect(
            partial(self.updateLabel,
                    header="z",
                    label=self.pose_z,
                    slider=self.height_slider))
        self.pose_command_layout.addWidget(self.pose_x, 0, 0)
        self.pose_command_layout.addWidget(self.pose_y, 0, 1)
        self.pose_command_layout.addWidget(self.pose_z, 0, 2)
        # Button to send the pose command to state machine as poseyaw event
        self.send_pose_command_button = QPushButton("Send Pose Command")
        self.send_pose_command_button.clicked.connect(
            self.poseCommandButtonCallback)
        self.pose_command_layout.addWidget(self.send_pose_command_button, 0, 3)
        self._additional_commands_layout.addWidget(self.pose_command_container)
        # Pose command container to store pose from Rviz and send to state
        # machine
        self.pose_command = None
        # Create Z velocity slider
        self.velocity_sliders = []
        self.velocity_command_labels = []
        self._additional_commands_layout.addWidget(
            QLabel('velocity Command (m/s), Yaw (deg)'))
        for i in range(3):
            self.velocity_sliders.append(
                self.createSlider(-20.0, 20.0, 0.0, 1.0))
            self._additional_commands_layout.addWidget(
                self.velocity_sliders[i])
        # Yaw
        self.velocity_sliders.append(
            self.createSlider(-180.0, 180.0, 0.0, 10.0))
        self._additional_commands_layout.addWidget(self.velocity_sliders[-1])
        # Add button for triggering velocity yaw
        self.velocity_command_container = QWidget()
        # Velocity command layout
        self.velocity_command_layout = QGridLayout()
        self.velocity_command_container.setLayout(self.velocity_command_layout)
        for i in range(3):
            # label to display velocity command from rviz to user
            self.velocity_command_labels.append(
                QLabel("v{0}: {1:.2f}".format(
                    i, self.velocity_sliders[i].value() / 10.0)))
            self.velocity_sliders[i].valueChanged.connect(
                partial(self.updateLabel,
                        header="v"+str(i),
                        label=self.velocity_command_labels[
                            i],
                        slider=self.velocity_sliders[i], scale=10.0))
            self.velocity_command_layout.addWidget(
                self.velocity_command_labels[i], 0, i)
        # Yaw
        self.velocity_command_labels.append(
            QLabel("Yaw: {0:.2f}".format(self.velocity_sliders[-1].value())))
        self.velocity_sliders[-1].valueChanged.connect(
            partial(self.updateLabel,
                    header="Yaw",
                    label=self.velocity_command_labels[
                        -1],
                    slider=self.velocity_sliders[-1]))
        self.velocity_command_layout.addWidget(
            self.velocity_command_labels[-1], 0, 3)
        # Button to send the pose command to state machine as poseyaw event
        self.send_velocity_command_button = QPushButton(
            "Send Velocity Command")
        self.send_velocity_command_button.clicked.connect(
            self.velocityCommandButtonCallback)
        self.velocity_command_layout.addWidget(
            self.send_velocity_command_button, 0, 4)
        self._additional_commands_layout.addWidget(
            self.velocity_command_container)

        self._container.addTab(self._status_tab, "StatusBasicCommands")
        self._container.addTab(
            self._additional_commands_tab, "AdditionalCommands")
        context.add_widget(self._container)

        # Add textboxes to update hooks from eventTrigger class
        # Define Partial callbacks
        systemStatusCallback = partial(
            self.updateStatus, text_box=self.system_status_textbox)
        # Connect Event Triggers
        self.event_trigger.status_signal.connect(
            systemStatusCallback)
        self.event_trigger.pose_command_signal.connect(
            self.poseCommandCallback)

    def createSlider(self, minimum, maximum, default_value, tick_interval):
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(minimum)
        slider.setMaximum(maximum)
        slider.setValue(default_value)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(tick_interval)
        return slider

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
        self.pose_x.setText("x: {0:.2f}".format(
            self.pose_command.pose.position.x))
        self.pose_y.setText("y: {0:.2f}".format(
            self.pose_command.pose.position.y))

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

    def velocityCommandButtonCallback(self):
        """
        Publishes stored velocity command
        """
        velocity_command = VelocityYaw()
        velocity_command.vx = self.velocity_sliders[0].value() / 10.0
        velocity_command.vy = self.velocity_sliders[1].value() / 10.0
        velocity_command.vz = self.velocity_sliders[2].value() / 10.0
        velocity_command.yaw = self.velocity_sliders[3].value() * np.pi / 180.0
        self.event_trigger.triggerVelocityCommand(velocity_command)

    def updateLabel(self, header, label, slider, scale=1):
        """
        Updates height label based on slider value
        """
        label.setText(header + ": {0:.2f}".format(slider.value() / scale))

    def updateStatus(self, status, text_box):
        """
        Generic placeholder function to update text box
        """
        if not sip.isdeleted(text_box):
            text_box.setHtml(status)
            doc_size = text_box.document().size()
            text_box.setFixedHeight(doc_size.height() + 10)
