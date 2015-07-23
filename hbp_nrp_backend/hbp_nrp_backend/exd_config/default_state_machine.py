"""
This module initializes a SMACH state machine with default states.
Subclass DefaultStateMachine for further use
"""

__author__ = 'PatrikScheidecker'

# This file is still wip, so there is no point in testing it

from . import OUTCOMES, OUTCOME_FINISHED
import rospy
import smach


def create_state_machine():
    """
    Instantiates a new instance of the state machine contained in this module

    :return: a SMACH state machine deriving from DefaultStateMachine
    """

    raise NotImplementedError("This method was not implemented in the concrete implementation")


class DefaultState(smach.State):  # pragma: no cover
    """
    Defines default state which terminates the state machine with positive outcome in case
    implementing state machines do not provide additional states
    """

    def __init__(self):
        """
        Initializes a default state with the only valid outcome "FINISHED"
        """

        smach.State.__init__(self, outcomes=[OUTCOME_FINISHED])

    def execute(self, ud):
        """
        The execution of the default state simply returns the positive outcome "FINISHED"
        """

        return OUTCOME_FINISHED


class DefaultStateMachine(object):  # pragma: no cover
    """
    DefaultStateMachine provides the basis for experiment controlling smach state machines to
     build upon. It initializes the smach state machine and configures it with the appropriate
     states and their transitions. Realizing subclasses are required to provide state machine
     states and transitions through a custom populate() implementation
    """

    def __init__(self):
        """
        Initializes a new DefaultStateMachine. Retrieves states and transitions through the
        populate method.
        """

        rospy.init_node("hbp_nrp_backend_experiment_state_machine")

        state_list = self.populate()

        # Create a SMACH state machine and populate it
        self.sm = smach.StateMachine(
            outcomes=OUTCOMES)

        with self.sm:
            for state_tuple in state_list:
                smach.StateMachine.add(
                    state_tuple[0],
                    state_tuple[1],
                    transitions=state_tuple[2])

    @staticmethod
    def populate():
        """
        Populates state machine with default states. This method is to be overwritten by subclasses
        to provide the purposeful states and transitions. Each state is identified by a unique
        name and produces a distinct outcome upon execution. For each state a dictionary specifies
        the name of the next state or the final outcome of the state machine in relation to the
        outcome of the state.

        Overwrite this function in state machines! The current implementation only initializes a
        DefaultState which returns "FINISHED" immediately after execution.

        :return: A list of (Name, smach.State, transitions={State Outcome: Name of New State, ...})
        tuples.
        """

        state_list = [('DEFAULT', DefaultState(), {'any': OUTCOME_FINISHED})]
        return state_list
        # returns state list in this format:
        # (name,State,transitions={}): [(,,{}), (,,{}), ...]

    def execute(self):
        """
        Executes the state machine and blocks until state machine has terminated.
        """

        result = self.sm.execute()
        rospy.loginfo(result)
