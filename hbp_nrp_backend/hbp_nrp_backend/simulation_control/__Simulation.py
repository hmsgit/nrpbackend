"""
This module contains the simulation class
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.simulation_control.__StateMachine import stateMachine
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException
from flask_restful import fields
from flask_restful_swagger import swagger
import datetime


@swagger.model
class Simulation(object):
    """
    The data class for simulations
    """
    def __init__(self, sim_id, experiment_id, owner, sim_gzserver_host, state='created'):
        """
        Creates a new simulation
        :param sim_id: The simulation id
        :param experiment_id: The experiment id (Path to ExD configuration)
        :param owner: The name of the user owning the simulation
        :param sim_gzserver_host: Denotes where the simulation will run once started. Set to
        'local' for localhost and 'lugano' for a dedicate machine on the Lugano viz cluster.
        :param state: The initial state (created by default)
        """
        self.__state = state
        self.__sim_id = sim_id
        self.__experiment_id = experiment_id
        self.__owner = owner
        self.__gzserver_host = sim_gzserver_host
        self.__creation_date = datetime.datetime.now().isoformat()
        self.__cle = None

        # The following two members are part of the fix for [NRRPLT-1899]:
        # We store the values of the left and right screen color in order to display
        # the correct color, whenever a simulation is loaded or joined. Gazebo itself
        # does not memorize the changed colors and hence we have this (ugly) workaround
        # as long as there is no better solution.
        self.__right_screen_color = 'Gazebo/Blue'
        self.__left_screen_color = 'Gazebo/Blue'

    # The next four methods are also part of the hack to fix [NRRPLT-1899].
    @property
    def right_screen_color(self):
        """
        Gets the right screen color
        """
        return self.__right_screen_color

    @right_screen_color.setter
    def right_screen_color(self, value):
        """
        Sets the right screen color
        """
        self.__right_screen_color = value

    @property
    def left_screen_color(self):
        """
        Gets the left screen color
        """
        return self.__left_screen_color

    @left_screen_color.setter
    def left_screen_color(self, value):
        """
        Sets the left screen color
        """
        self.__left_screen_color = value

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
    def creation_date(self):
        """
        The creation date of this simulation
        :return: The creation date
        """
        return self.__creation_date

    @property
    def state(self):
        """
        Gets the state of the simulation.
        If the simulation CLE object is created then we do trust the CLE more than
        ourself and query it !
        """
        if self.__cle is not None:
            try:
                self.__state = self.__cle.get_simulation_state()
            except ROSCLEClientException:
                # If anything goes wrong, we assume that we are in the "stopped" state
                self.__state = "stopped"
        return self.__state

    # pylint: disable=broad-except
    @state.setter
    def state(self, new_state):
        """
        Sets the state of the simulation to the given value
        :param new_state: The new state
        """
        transitions = stateMachine[self.__state]
        if new_state not in transitions:
            raise InvalidStateTransitionException()
        try:
            transitions[new_state](self.__sim_id)
            self.__state = new_state
        except Exception as e:
            self.__state = "failed"
            raise e

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
    def gzserver_host(self):
        """
        Gets the host/cluster where the gzserver is running or will run once the simulation is
        started.
        """
        return self.__gzserver_host

    resource_fields = {
        'state': fields.String,
        'simulationID': fields.Integer(attribute='sim_id'),
        'experimentID': fields.String(attribute='experiment_id'),
        'owner': fields.String(attribute='owner'),
        'creationDate': fields.String(attribute='creation_date'),
        'gzserverHost': fields.String(attribute='gzserver_host'),
        'right_screen_color': fields.String(attribute='right_screen_color'),
        'left_screen_color': fields.String(attribute='left_screen_color')
    }
    required = ['state', 'simulationID', 'experimentID', 'gzserverHost']


class InvalidStateTransitionException(Exception):
    """
    Represents that an invalid state transition was attempted
    """
    pass
