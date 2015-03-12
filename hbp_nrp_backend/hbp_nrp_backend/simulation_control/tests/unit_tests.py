"""
This file loads the unit test transitions
"""

from hbp_nrp_backend.simulation_control import __StateMachine as sm


__author__ = 'GeorgHinkel'

last_sim_id = None
last_transition = None


def __start_simulation(sim_id):
    """
    Starts the simulation with the given id
    :param sim_id: The simulation id
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim_id
    last_transition = "start"


def __pause_simulation(sim_id):
    """
    Pauses the simulation with the given id
    :param sim_id: The simulation id
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim_id
    last_transition = "pause"


def __reset_simulation(sim_id):
    """
    Resumes the simulation with the given simulation id
    :param sim_id: The simulation id
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim_id
    last_transition = "reset"


def __stop_simulation(sim_id):
    """
    Stops the simulation with the given id
    :param sim_id: The simulation id
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim_id
    last_transition = "stop"


def __initialize_simulation(sim_id):
    """
    Releases the simulation with the given id
    :param sim_id: The simulation id
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim_id
    last_transition = "initialize"


def use_unit_test_transitions():
    """
    Defines that unit test transition should be used
    """
    sm.pause_simulation = __pause_simulation
    sm.start_simulation = __start_simulation
    sm.reset_simulation = __reset_simulation
    sm.stop_simulation = __stop_simulation
    sm.initialize_simulation = __initialize_simulation


def start_will_raise_exception(exception):
    # Dark magic taken from
    # http://stackoverflow.com/questions/8294618/define-a-lambda-expression-that-raises-an-exception
    sm.start_simulation = lambda x: (_ for _ in ()).throw(exception)
