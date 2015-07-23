#!/usr/bin/env python
"""
This file is a state-machine prototype for the husky demo.
In the future, this will be replaced with an auto-generated
script from the experiment configuration.

Usage: import default_state_machine, create a subclass of
DefaultStateMachine and overwrite the function populate()
"""

__author__ = 'PatrikScheidecker'

# This file is still wip, so there is no point in testing it

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
    """
    State machine generated for the following event of an ExD configuration:

    .. code-block:: xml

        <event>
         <conditions>
          <condition>
           <!-- trigger after 30 seconds of simulation time -->
           <timeline>
            <time>00:00:90</time>
           </timeline>
          </condition>
         </conditions>
         <actions>
          <!-- set screen color to red -->
          <property>
           <identifier>model:right_vr_screen/link:body/visual:screen_glass/material/script/uri
           </identifier>
           <value>Gazebo/Red</value>
          </property>
         </actions>
        </event>
    """

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


def main():  # pragma: no cover
    """
    This is the main method
    """

    statemachine = StateMachineTimedColorChange()
    statemachine.execute()


if __name__ == "__main__":  # pragma: no cover
    main()
