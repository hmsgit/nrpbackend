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
clean = pt.clean


def __start_simulation(simulation):
    """
    Wraps the call to the variable function

    :param simulation: The simulation id
    """
    start_simulation(simulation)


def __pause_simulation(simulation):
    """
    Wraps the call to the variable function

    :param simulation: The simulation id
    """
    pause_simulation(simulation)


def __reset_simulation(simulation):
    """
    Wraps the call to the variable function

    :param simulation: The simulation id
    """
    reset_simulation(simulation)


def __stop_simulation(simulation):
    """
    Wraps the call to the variable function

    :param simulation: The simulation
    """
    stop_simulation(simulation)


def __initialize_simulation(simulation):
    """
    Wraps the call to the variable function

    :param simulation: The simulation
    """
    initialize_simulation(simulation)


def __clean(simulation):
    """
    Wraps the call to the variable function

    :param simulation: The simulation
    """
    clean(simulation)


stateMachine = {'created': {'initialized': __initialize_simulation,
                            'failed': __clean},
                'initialized': {'started': __start_simulation,
                                'stopped': __stop_simulation,
                                'halted': __clean},
                'started': {'paused': __pause_simulation,
                            'initialized': __reset_simulation,
                            'stopped': __stop_simulation,
                            'halted': __clean},
                'paused': {'started': __start_simulation,
                           'initialized': __reset_simulation,
                           'stopped': __stop_simulation,
                           'halted': __clean},
                'halted': {'failed': __stop_simulation},
                'stopped': {},
                'failed': {}}

reroutes = {'halted': {'stopped': 'failed'}}

fail_states = {'created': 'failed',
               'initialized': 'halted',
               'started': 'halted',
               'paused': 'halted',
               'halted': 'failed'}
