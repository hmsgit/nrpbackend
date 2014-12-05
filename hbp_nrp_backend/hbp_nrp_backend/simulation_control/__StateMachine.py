"""
Implementations for the state transitions
"""
from hbp_nrp_backend.simulation_control import transitions as pt

__author__ = 'GeorgHinkel'

start_simulation = pt.start_simulation
pause_simulation = pt.pause_simulation
reset_simulation = pt.reset_simulation
stop_simulation = pt.stop_simulation
initialize_simulation = pt.initialize_simulation


def __start_simulation(sim_id):
    """
    Wraps the call to the variable function
    :param sim_id: The simulation id
    """
    start_simulation(sim_id)


def __pause_simulation(sim_id):
    """
    Wraps the call to the variable function
    :param sim_id: The simulation id
    """
    pause_simulation(sim_id)


def __reset_simulation(sim_id):
    """
    Wraps the call to the variable function
    :param sim_id: The simulation id
    """
    reset_simulation(sim_id)


def __stop_simulation(sim_id):
    """
    Wraps the call to the variable function
    :param sim_id: The simulation id
    """
    stop_simulation(sim_id)


def __initialize_simulation(sim_id):
    """
    Wraps the call to the variable function
    :param sim_id: The simulation id
    """
    initialize_simulation(sim_id)


stateMachine = {'created': {'initialized': __initialize_simulation},
                'initialized': {'started': __start_simulation},
                'started': {'paused': __pause_simulation,
                            'initialized': __reset_simulation,
                            'stopped': __stop_simulation},
                'paused': {'started': __start_simulation,
                           'initialized': __reset_simulation,
                           'stopped': __stop_simulation},
                'stopped': {}}
