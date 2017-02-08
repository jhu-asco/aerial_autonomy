#!/usr/bin/env python
"""
Generate a GUI to trigger events for state machine

@author: gowtham
"""
import argparse
from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import (QLabel, QVBoxLayout,
                                         QGridLayout, QWidget,
                                         QTextEdit, QPushButton)
from ros_event_trigger import RosEventTrigger
from argparse import ArgumentParser
from functools import partial

# %%


class EventTransmissionGUI(Plugin):

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

        # Set Layout
        self._container = QWidget()
        self._container.setWindowTitle(self.event_trigger.event_manager_name)
        self._layout = QVBoxLayout()
        self._container.setLayout(self._layout)

        # Create Textboxes and add to Layout
        self._layout.addWidget(QLabel('State Machine state'))
        self.state_machine_textbox = QTextEdit()
        self.state_machine_textbox.setReadOnly(True)
        self._layout.addWidget(self.state_machine_textbox)

        self._layout.addWidget(QLabel('Quad Status'))
        self.quad_textbox = QTextEdit()
        self.quad_textbox.setReadOnly(True)
        self._layout.addWidget(self.quad_textbox)

        if args.use_arm:
            self._layout.addWidget(QLabel('Arm Status'))
            self.arm_textbox = QTextEdit()
            self.arm_textbox.setReadOnly(True)
            self._layout.addWidget(self.arm_textbox)

        # Define and connect buttons
        self._layout.addWidget(QLabel('Event Triggers'))
        self.button_container = QWidget()
        self.push_buttons = list()
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
        stateMachineStatusCallback = partial(
            self.updateStatus, text_box=self.state_machine_textbox)
        quadStatusCallback = partial(
            self.updateStatus, text_box=self.quad_textbox)
        # Connect Event Triggers
        self.event_trigger.state_machine_signal.connect(
            stateMachineStatusCallback)
        self.event_trigger.quad_signal.connect(quadStatusCallback)
        # Same for arm
        if args.use_arm:
            armStatusCallback = partial(
                self.updateStatus, text_box=self.arm_textbox)
            self.event_trigger.arm_signal.connect(armStatusCallback)

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
        group.add_argument("--use_arm", action='store_true',
                           help="To add arm state textbox")

    def get_row_col(self, button_index, ncols):
        """
        Automatically find the row and col to add the button
        to in a grid based on index of the button
        """
        col_index = button_index % ncols
        row_index = int((button_index - col_index) / ncols)
        return(row_index, col_index)

    def updateStatus(self, status, text_box):
        """
        Generic placeholder function to update text box
        """
        text_box.setText(status)
