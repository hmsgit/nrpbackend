"""
This file loads the unit test transitions
"""

from hbp_nrp_backend.simulation_control import __StateMachine as sm


__author__ = 'GeorgHinkel'

last_sim_id = None
last_transition = None

__original_start = sm.start_simulation
__original_pause = sm.pause_simulation
__original_reset = sm.reset_simulation
__original_initialize = sm.initialize_simulation
__original_stop = sm.stop_simulation
__original_clean = sm.clean


def __start_simulation(sim):
    """
    Starts the simulation

    :param sim: The simulation id
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim.sim_id
    last_transition = "start"


def __pause_simulation(sim):
    """
    Pauses the simulation

    :param sim: The simulation
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim.sim_id
    last_transition = "pause"


def __reset_simulation(sim):
    """
    Resumes the simulation

    :param sim: The simulation
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim.sim_id
    last_transition = "reset"


def __stop_simulation(sim):
    """
    Stops the simulation

    :param sim: The simulation id
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim.sim_id
    last_transition = "stop"


def __initialize_simulation(sim):
    """
    Releases the simulation

    :param sim: The simulation
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim.sim_id
    last_transition = "initialize"

def __clean(sim):
    """
    Cleans the simulation

    :param sim: The simulation
    """
    global last_sim_id
    global last_transition
    last_sim_id = sim.sim_id
    last_transition = "clean"


def use_unit_test_transitions():
    """
    Defines that unit test transition should be used
    """
    sm.pause_simulation = __pause_simulation
    sm.start_simulation = __start_simulation
    sm.reset_simulation = __reset_simulation
    sm.stop_simulation = __stop_simulation
    sm.initialize_simulation = __initialize_simulation
    sm.clean = __clean

def use_production_transitions():
    """
    Switches back to production transitions
    """
    sm.pause_simulation = __original_pause
    sm.start_simulation = __original_start
    sm.reset_simulation = __original_reset
    sm.stop_simulation = __original_stop
    sm.initialize_simulation = __original_initialize
    sm.clean = __original_clean

def start_will_raise_exception(exception):
    # Dark magic taken from
    # http://stackoverflow.com/questions/8294618/define-a-lambda-expression-that-raises-an-exception
    sm.start_simulation = lambda x: (_ for _ in ()).throw(exception)
