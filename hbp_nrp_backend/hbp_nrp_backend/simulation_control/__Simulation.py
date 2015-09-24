"""
This module contains the simulation class
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.simulation_control.__StateMachine import stateMachine
from hbp_nrp_backend.experiment_control.__ExperimentStateMachine import \
    ExperimentStateMachineInstance
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException
from flask_restful import fields
from flask_restful_swagger import swagger
import datetime


@swagger.model
class Simulation(object):
    """
    The data class for simulations
    """
    def __init__(self, sim_id, experiment_conf, environment_conf, owner, sim_gzserver_host,
                 sim_operation_mode='view', state='created'):
        """
        Creates a new simulation

        :param sim_id: The simulation id
        :param experiment_conf: The experiment configuration (Path to ExD configuration)
        :param environment_conf: The environment configuration (Path to environment configuration)
        :param owner: The name of the user owning the simulation
        :param sim_gzserver_host: Denotes where the simulation will run once started. Set to
        'local' for localhost and 'lugano' for a dedicate machine on the Lugano viz cluster.
        :param sim_operation_mode: Denotes whether the simulation should be started in 'edit' or \
        'view' mode
        :param state: The initial state (created by default)
        """
        self.__state = state
        self.__sim_id = sim_id
        self.__experiment_conf = experiment_conf
        self.__environment_conf = environment_conf
        self.__owner = owner
        self.__gzserver_host = sim_gzserver_host
        self.__operation_mode = sim_operation_mode
        self.__creation_datetime = datetime.datetime.now()
        self.__cle = None
        self.__state_machines = dict()
        self.__errors = 0  # We use that for monitoring

        # The following two members are part of the fix for [NRRPLT-1899]:
        # We store the values of the left and right screen color in order to display
        # the correct color, whenever a simulation is loaded or joined. Gazebo itself
        # does not memorize the changed colors and hence we have this (ugly) workaround
        # as long as there is no better solution.
        self.__right_screen_color = 'Gazebo/Blue'
        self.__left_screen_color = 'Gazebo/Blue'

    @property
    def errors(self):
        """
        :return: the number of errors for the current simulation
        """
        return self.__errors

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
    def experiment_conf(self):
        """
        Gets the experiment configuration, i.e. the path to the ExD configuration

        :return: Experiment configuration
        :rtype: string
        """
        return self.__experiment_conf

    @property
    def environment_conf(self):
        """
        Gets the environment configuration, i.e. the path to the environment configuration
        """
        return self.__environment_conf

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
    def creation_datetime(self):
        """
        The creation datetime of this simulation
        :return: The creation datetime
        """
        return self.__creation_datetime

    @property
    def creation_date(self):
        """
        The creation date of this simulation

        :return: The creation date
        """
        return self.__creation_datetime.isoformat()

    @property
    def operation_mode(self):
        """
        The operation mode of this simulation

        :return: The operation mode ('view' or 'edit')
        """
        return self.__operation_mode

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
                self.__errors += 1
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
            self.__errors += 1
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
    def state_machines(self):
        """
        Get state_machines dictionary

        :return: The dictionary of state machine instances
        """
        return self.__state_machines

    @state_machines.setter
    def state_machines(self, state_machines):
        """
        Set state_machine dictionary

        :param state_machines: The new state machines dictionary
        """
        self.__state_machines = state_machines

    def get_state_machine_code(self, name):
        """
        Get state machine code

        :param name: name of the state machine
        :return: source code of the state machine (string) or False (bool) when \
        state machine is not found
        """
        if name in self.state_machines:
            return self.state_machines[name].sm_source

        return False

    def set_state_machine_code(self, name, python_code):
        """
        Set state machine code.
        Only works, when simulation is in state "initialized"

        :param   name:        name of the state machine
        :type    name:        string
        :param   python_code: source code of the state machine
        :type    python_code: string
        :raise:  NameError, SyntaxError, AttributeError, ...
        """
        assert self.state == 'initialized'

        sm = None
        if name in self.state_machines:
            sm = self.state_machines[name]
            sm.sm_source = python_code
        else:
            sm = ExperimentStateMachineInstance(name, python_code)
            self.state_machines[name] = sm

        sm.initialize_sm()

    def delete_state_machine(self, name):
        """
        Delete state machine.

        :param   name:        name of the state machine
        :type    name:        string
        :return: (pair) --    A pair made of a
                              (bool) boolean, which is True on success, False otherwise, and a
                              (string) string indicating either a successful delete operation
                              or that the state machine was not found
        :rtype:  bool, string
        :raise:  NameError
        """

        sm = self.state_machines.pop(name, None)
        if sm is not None:
            assert isinstance(sm, ExperimentStateMachineInstance)
            if sm.is_running:
                sm.request_termination()
                sm.wait_termination()
            return True, "Success"

        return False, "State machine '{0}' not found.".format(name)

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
        'experimentConfiguration': fields.String(attribute='experiment_conf'),
        'environmentConfiguration': fields.String(attribute='environment_conf'),
        'owner': fields.String(attribute='owner'),
        'creationDate': fields.String(attribute=lambda x: x.creation_date),
        'gzserverHost': fields.String(attribute='gzserver_host'),
        'operationMode': fields.String(attribute='operation_mode'),
        'right_screen_color': fields.String(attribute='right_screen_color'),
        'left_screen_color': fields.String(attribute='left_screen_color')
    }
    required = ['state', 'simulationID', 'experimentConfiguration', 'gzserverHost', 'operationMode']


class InvalidStateTransitionException(Exception):
    """
    Represents that an invalid state transition was attempted
    """
    pass


class InvalidTransferFunctionCodeException(Exception):
    """
    Represents that invalid source code for transfer function was submitted
    """
    pass
