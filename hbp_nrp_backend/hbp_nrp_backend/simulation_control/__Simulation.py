"""
This module contains the simulation class
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.simulation_control.__StateMachine import stateMachine
from flask_restful import fields
from flask_restful_swagger import swagger
from threading import Timer
import logging
import time

logger = logging.getLogger(__name__)


@swagger.model
class Simulation(object):
    """
    The data class for simulations
    """
    def __init__(self, sim_id, experiment_id, owner, state='created'):
        """
        Creates a new simulation
        :param sim_id: The simulation id
        :param experiment_id: The experiment id (Path to ExD configuration)
        :param owner: The name of the user owning the simulation
        :param state: The initial state (created by default)
        """
        self.__state = state
        self.__sim_id = sim_id
        self.__experiment_id = experiment_id
        self.__owner = owner
        self.__cle = None
        self.__timeout = None
        self.__expiring = False
        self.__starting_time = None
        self.__timer = None

    @property
    def experiment_id(self):
        """
        Gets the experiment ID, i.e. the path to the ExD configuration
        """
        return self.__experiment_id

    @property
    def sim_id(self):
        """
        Gets the simulation ID
        """
        return self.__sim_id

    @property
    def owner(self):
        """
        The owner of this simulation
        :return: The owner name
        """
        return self.__owner

    @property
    def state(self):
        """
        Gets the state of the simulation
        """
        return self.__state

    @state.setter
    def state(self, new_state):
        """
        Sets the state of the simulation to the given value
        :param new_state: The new state
        """
        transitions = stateMachine[self.__state]
        if new_state not in transitions:
            raise InvalidStateTransitionException()
        transitions[new_state](self.__sim_id)
        self.__state = new_state

    @property
    def cle(self):
        """
        The CLE for this simulation
        :return: The CLE instance
        """
        return self.__cle

    @cle.setter
    def cle(self, cle):
        """
        Sets the CLE for this simulation
        :param cle: The new CLE
        """
        self.__cle = cle

    @property
    def timeout(self):
        """
        The timeout time of this simulation
        :return: The timeout time
        """
        return self.__timeout

    @timeout.setter
    def timeout(self, timeout):
        """
        Sets the timeout time for this simulation, default is 300
        :param cle: The new timeout time
        """
        if timeout is None:
            self.__timeout = 300.0
        elif timeout <= 0.0:
            self.__timeout = 300.0
        else:
            self.__timeout = timeout

    def get_remaining_time(self):
        """
        Gets the remaining time of the simulation
        :return: the remaining time
        """
        if (not self.__expiring):
            return self.__timeout
        time_passed = time.time() - self.__starting_time
        remaining = self.__timeout - time_passed
        if (remaining <= 0.0):
            return 0.0
        return remaining

    def start_timeout(self):
        """
        Start the timeout on the current simulation
        """
        self.__timer = Timer(self.__timeout, self.quit_by_timeout)
        self.__timer.setDaemon(True)
        self.__timer.start()
        self.__expiring = True
        self.__starting_time = time.time()
        logger.info("Simulation will timeout in %f seconds", self.__timeout)

    def stop_timeout(self):
        """
        Stop the timeout
        """
        if self.__expiring:
            self.__timer.cancel()
            self.__expiring = False
            logger.info("Timeout cancelled")

    def quit_by_timeout(self):
        """
        Stops the simulation
        """
        self.state = 'stopped'
        logger.info("Force quitting the simulation")

    resource_fields = {
        'state': fields.String,
        'simulationID': fields.Integer(attribute='sim_id'),
        'experimentID': fields.String(attribute='experiment_id'),
        'owner': fields.String(attribute='owner')
    }
    required = ['state', 'simulationID', 'experimentID']


class InvalidStateTransitionException(Exception):
    """
    Represents that an invalid state transition was attempted
    """
    pass
