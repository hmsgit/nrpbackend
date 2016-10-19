import smach_ros
from smach import StateMachine
from hbp_nrp_excontrol.nrp_states import WaitForClientLogState, SetMaterialColorServiceState

sm = StateMachine(outcomes=['FINISHED', 'ERROR', 'PREEMPTED'])

with sm:
    StateMachine.add(
        "wait_for_red_log",
        WaitForClientLogState('left_tv_red'),
        transitions={'valid': 'wait_for_red_log',
                     'invalid': 'set_left_screen_red',
                     'preempted': 'PREEMPTED'}
    )

    StateMachine.add(
        "set_left_screen_red",
        SetMaterialColorServiceState('left_vr_screen',
                                     'body',
                                     'screen_glass',
                                     'Gazebo/Red'),
        transitions={'succeeded': 'wait_for_blue_log',
                     'aborted': 'ERROR',
                     'preempted': 'PREEMPTED'}
    )

    StateMachine.add(
        "wait_for_blue_log",
        WaitForClientLogState('left_tv_blue'),
        transitions={'valid': 'wait_for_blue_log',
                     'invalid': 'set_left_screen_blue',
                     'preempted': 'PREEMPTED'}
    )

    StateMachine.add(
        "set_left_screen_blue",
        SetMaterialColorServiceState('left_vr_screen',
                                     'body',
                                     'screen_glass',
                                     'Gazebo/Blue'),
        transitions={'succeeded': 'wait_for_red_log',
                     'aborted': 'ERROR',
                     'preempted': 'PREEMPTED'}
    )
