"""
This module initializes a SMACH state machine with default states.
Subclass DefaultStateMachine for further use
"""

__author__ = 'PatrikScheidecker'

# This file is still wip, so there is no point in testing it

import rospy
import smach


class DefaultState(smach.State):  # pragma: no cover
    """
    Defines default state
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['FINISHED'])

    def execute(self, ud):
        return 'FINISHED'


class DefaultStateMachine(object):  # pragma: no cover
    """
    Defines default state machine
    """

    def __init__(self):
        rospy.init_node("hbp_nrp_backend_experiment_state_machine")

        state_list = self.populate()

        # Create a SMACH state machine and populate it
        self.sm = smach.StateMachine(
            outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
                      'ACTION_PREEMPTED', 'ACTION_ERROR'])
        with self.sm:
            for state_tuple in state_list:
                smach.StateMachine.add(
                    state_tuple[0],
                    state_tuple[1],
                    transitions=state_tuple[2])

    @staticmethod
    def populate():
        """
        Populates state machine with default states

        Overwrite this function in state machines
        """

        state_list = [('DEFAULT', DefaultState(), {'any': 'FINISHED'})]
        return state_list
        # returns state list in this format:
        # (name,State,transitions={}): [(,,{}), (,,{}), ...]

    def execute(self):
        """
        Executes Smach plan
        """

        result = self.sm.execute()
        rospy.loginfo(result)
