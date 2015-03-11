#!/usr/bin/env python
"""
This file is a state-machine prototype for the husky demo.
In the future, this will be replaced with an auto-generated
script from the experiment configuration.

Usage: import default_state_machine, create a subclass of
DefaultStateMachine and overwrite the function populate()
"""

__author__ = 'PatrikScheidecker'

import smach_ros

from hbp_nrp_backend.exd_config.default_state_machine import DefaultStateMachine

from std_msgs.msg import Time
from gazebo_msgs.srv import SetVisualProperties
from gazebo_msgs.srv import SetVisualPropertiesRequest


def timeline_condition_cb(user_data, time):
    """
    Callback to check timeline for given time stamp

    Note: time stamps is converted to seconds

    Note: MonitorState is designed to block while condition is true (e.g. time stamp greater
    than or equal 90 seconds), but we need it to block while condition is not true (e.g. time
    stamp smaller than 90 seconds).
    """

    if time.clock.secs < 90:
        return True  # condition not met so far
    else:
        return False  # condition met, evaluates state outcome to 'invalid'


class StateMachineTimedColorChange(DefaultStateMachine):
    """
    State machine generated for the following event of an ExD configuration:

    >>> Example of an experiment configuration:
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

        # Use max_checks=2 to terminate MonitorState after each condition evaluation (otherwise no
        # preemption possible, as state blocks while condition is true). Depending on the publishing
        # frequency of the topic or simulation frequency max_checks could more like be 10 or 100 to
        # terminate MonitorState roughly once per second.
        state_tuple = 'CONDITION', smach_ros.MonitorState('/clock', Time,
                                                          timeline_condition_cb, 2), \
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


def main():
    """
    This is the main method
    """
    statemachine = StateMachineTimedColorChange()
    statemachine.execute()


if __name__ == "__main__":
    main()
