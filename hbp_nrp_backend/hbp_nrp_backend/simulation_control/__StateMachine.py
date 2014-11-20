"""
Implementations for the state transitions
"""
from hbp_nrp_backend.simulation_control import transitions as pt

__author__ = 'GeorgHinkel'

start_simulation = pt.start_simulation
pause_simulation = pt.pause_simulation
resume_simulation = pt.resume_simulation
stop_simulation = pt.stop_simulation
release_simulation = pt.release_simulation


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


def __resume_simulation(sim_id):
    """
    Wraps the call to the variable function
    :param sim_id: The simulation id
    """
    resume_simulation(sim_id)


def __stop_simulation(sim_id):
    """
    Wraps the call to the variable function
    :param sim_id: The simulation id
    """
    stop_simulation(sim_id)


def __release_simulation(sim_id):
    """
    Wraps the call to the variable function
    :param sim_id: The simulation id
    """
    release_simulation(sim_id)


stateMachine = {'created': {'started': __start_simulation},
                'started': {'paused': __pause_simulation, 'stopped': __stop_simulation},
                'paused': {'resumed': __resume_simulation, 'stopped': __stop_simulation},
                'resumed': {'paused': __pause_simulation, 'stopped': __stop_simulation},
                'stopped': {'released': __release_simulation},
                'released': {}}
