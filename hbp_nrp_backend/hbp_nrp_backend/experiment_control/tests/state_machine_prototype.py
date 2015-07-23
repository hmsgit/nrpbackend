#!/usr/bin/env python
"""
This file is a copy of the state-machine prototype used for the husky demo. The original file
can be found in the package hbp_nrp_backend.exd_config. The purpose of this copy is to serve as
test object for testing the experiment state machine instance implementation
"""

__author__ = 'PatrikScheidecker'

import smach_ros

from hbp_nrp_backend.exd_config.default_state_machine import DefaultStateMachine

from std_msgs.msg import Time
from gazebo_msgs.srv import SetVisualProperties
from gazebo_msgs.srv import SetVisualPropertiesRequest


def create_state_machine():
    """
    Instantiates a new instance of the state machine contained in this module

    :return: a SMACH state machine deriving from DefaultStateMachine
    """

    return StateMachineTimedColorChange()


def timeline_condition_cb(user_data, time):  # pragma: no cover
    """
    Callback to check timeline for given time stamp

    Note: time stamps is converted to seconds

    Note: MonitorState is designed to block while condition is true (e.g. time stamp greater
    than or equal 90 seconds), but we need it to block while condition is not true (e.g. time
    stamp smaller than 90 seconds).
    """

    return time.clock.secs < 90


class StateMachineTimedColorChange(DefaultStateMachine):  # pragma: no cover

    @staticmethod
    def populate():
        state_list = []
        # add a MonitorState to wait for condition
        # MonitorState(topic, msg_type, cond_cb, max_checks)

        # The callback method is called once per incoming message on the specified ros topic.
        # max_checks determines how many times the callback method has to evaluate to True before
        # the state returns outcome "valid". With max_checks set to -1 the state does not produce
        # outcome "valid" but remains in the state until the callback method evaluates to False or
        # the state is preempted. -1 is the default value if there is no max_checks argument
        # provided.
        state_tuple = 'CONDITION', smach_ros.MonitorState('/clock', Time,
                                                          timeline_condition_cb), \
                      {'valid': 'CONDITION', 'invalid': 'ACTION',
                       'preempted': 'CONDITION_PREEMPTED'}
        state_list.append(state_tuple)

        # add a ServiceState to call the '/gazebo/set_visual_properties' service
        state_tuple = 'ACTION', smach_ros.ServiceState('/gazebo/set_visual_properties',
                                                       SetVisualProperties,
                                                       request=SetVisualPropertiesRequest(
                                                           'right_vr_screen',
                                                           'body',
                                                           'screen_glass',
                                                           'material:script:name',
                                                           'Gazebo/Red')), \
                      {'succeeded': 'FINISHED', 'aborted': 'ACTION_ERROR',
                       'preempted': 'ACTION_PREEMPTED'}
        state_list.append(state_tuple)
        return state_list
