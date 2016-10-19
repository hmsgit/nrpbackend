import smach_ros
from smach import StateMachine
from hbp_nrp_excontrol.nrp_states import RobotPoseMonitorState
from hbp_nrp_excontrol.logs import clientLogger

clientLogger.info("Start SM")


def robot_pose_cb(user_data, state):
    clientLogger.info("pos: x: " + str(state.position.x) +
                      ", y: " + str(state.position.y) +
                      ", z: " + str(state.position.z))

    return True

sm = StateMachine(outcomes=['FINISHED', 'ERROR', 'PREEMPTED'])
with sm:
    StateMachine.add('initial_condition',
                     RobotPoseMonitorState(robot_pose_cb),
                     transitions={'valid': 'initial_condition',
                                  'invalid': 'initial_condition',
                                  'preempted': 'PREEMPTED'})
