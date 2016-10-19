import smach_ros
from smach import StateMachine
from hbp_nrp_excontrol.nrp_states import RobotPoseMonitorState, ClientLogState

sm = StateMachine(outcomes=['FINISHED', 'ERROR', 'PREEMPTED'])

with sm:
    StateMachine.add(
        "wait_for_husky_left",
        RobotPoseMonitorState(lambda ud, p: not ((-1 < p.position.x < 1) and
                    (-2.5 < p.position.y < -1.8) and
            (0 < p.position.z < 1))),
        transitions={'valid': 'wait_for_husky_left',
                     'invalid': 'log_husky_at_left',
                     'preempted': 'PREEMPTED'}
    )

    StateMachine.add(
        "log_husky_at_left",
        ClientLogState("Husky is at the LEFT!"),
        transitions={'succeeded': 'wait_for_husky_right',
                     'aborted': 'ERROR'}
    )

    StateMachine.add(
        "wait_for_husky_right",
        RobotPoseMonitorState(lambda ud, p: not ((-1 < p.position.x < 1) and
                    (1.8 < p.position.y < 2.5) and
            (0 < p.position.z < 1))),
        transitions={'valid': 'wait_for_husky_right',
                     'invalid': 'log_husky_at_right',
                     'preempted': 'PREEMPTED'}
    )

    StateMachine.add(
        "log_husky_at_right",
        ClientLogState("Husky is at the RIGHT!"),
        transitions={'succeeded': 'wait_for_husky_left',
                     'aborted': 'ERROR'}
    )
