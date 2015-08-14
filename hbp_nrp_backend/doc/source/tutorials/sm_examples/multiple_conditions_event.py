#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String


def condition_30_cb(user_data, status):
    # ... parse JSON in status message ...
    # return TRUE as long as condition is not met
    return status.simulationTime < 30


def condition_spike_cb(user_data, spike):
    # ... parse JSON in spike message ...
    # return TRUE as long as condition is not met
    return spike < 10


if __name__ == "__main__":
    # Create a ROS node for this state machine
    rospy.init_node("hbp_nrp_backend_sm_exp_control")

    sm = smach.Concurrence(outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED'],
                           default_outcome='ERROR', input_keys=[], output_keys=[],
                           outcome_map = {'FINISHED':{'CONDITION_30':'invalid', 'CONDITION_SPIKE':'invalid'},
                                     'CONDITION_PREEMPTED':{'CONDITION_30':'preempted'},
                                     'CONDITION_PREEMPTED':{'CONDITION_SPIKE':'preempted'}})

    with sm:
        smach.Concurrence.add('CONDITION_30', smach_ros.MonitorState('/ros_cle_simulation/status',
                                                      String,
                                                      condition_30_cb))

        smach.Concurrence.add('CONDITION_SPIKE', smach_ros.MonitorState('/monitoring/left_wheel_neuron_rate_monitor',
                                                      String,
                                                      condition_spike_cb))

    result = sm.execute()
    rospy.loginfo(result)
