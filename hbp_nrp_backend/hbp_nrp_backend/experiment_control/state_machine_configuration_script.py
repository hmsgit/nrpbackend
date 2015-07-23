"""
This file provides convenience method to interact with sets of experiment controlling state
machines
"""
__author__ = 'Sebastian Krach'

from hbp_nrp_backend.experiment_control import ExperimentStateMachineInstance, MAX_TIMEOUT, \
    ConcurrentExecutionException


def initialize_state_machines(sm_paths=None, sm_instances=None):
    """
    Initializes a set of state machines to control the experiment execution.

    :param sm_paths: A dictionary specifying the state machine id (sm_id) and the path to the smach
    python script
    :return: An dictionary of initialized ExperimentStateMachine instances suitable for execution
    """
    if sm_instances is None:
        sm_instances = {}

    if sm_paths is not None:
        for key in sm_paths:
            with open(sm_paths[key], "r") as sm_source_file:
                sm_source = sm_source_file.readlines()
                sm_source = "".join(sm_source)
                sm_instances[key] = ExperimentStateMachineInstance(key, sm_source)

    for sm in sm_instances.itervalues():
        sm.initialize_sm()

    return sm_instances


def start_state_machines(state_machines, fail_if_running=True):
    """
    Convenience method that starts the execution of all state machines in the passed dictionary.

    :param state_machines: a dictionary of (sm_id: ExperimentStateMachineInstance) tuples
    """

    for sm in state_machines.itervalues():
        if not sm.is_running():
            sm.start_execution()
        else:
            if fail_if_running:
                raise ConcurrentExecutionException("The state machine %s is already running" %
                                                   str(sm))


def request_sm_termination(state_machines):
    """
    Convenience method to request termination of all state machines in the passed dictionary.

    :param state_machines: a dictionary of (sm_id: ExperimentStateMachineInstance) tuples
    """

    for sm in state_machines.itervalues():
        sm.request_termination()


def wait_sm_termination(state_machines, timeout=MAX_TIMEOUT):
    """
    Convenience method that waits upon the termination of all state machines in the passed
    dictionary.

    :param state_machines: a dictionary of (sm_id: ExperimentStateMachineInstance) tuples
    """

    for sm in state_machines.itervalues():
        sm.wait_termination(timeout)
