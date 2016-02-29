"""
This module contains the simulation class
"""

__author__ = 'Georg Hinkel'

from hbp_nrp_backend.simulation_control.__StateMachine import stateMachine, reroutes
from hbp_nrp_excontrol.StateMachineManager import StateMachineManager
from hbp_nrp_excontrol.StateMachineInstance import StateMachineInstance
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException
from tempfile import NamedTemporaryFile
from flask_restful import fields
from flask_restful_swagger import swagger
from pytz import timezone
import datetime
import logging

logger = logging.getLogger(__name__)


@swagger.model
class Simulation(object):
    """
    The data class for simulations
    """
    # pylint: disable=too-many-arguments
    def __init__(self, sim_id, experiment_conf, environment_conf, owner, sim_gzserver_host,
                 context_id=None, sim_operation_mode='view', state='created'):
        """
        Creates a new simulation

        :param sim_id: The simulation id
        :param experiment_conf: The experiment configuration (Path to ExD configuration)
        :param environment_conf: The environment configuration (Path to environment configuration)
        :param owner: The name of the user owning the simulation
        :param sim_gzserver_host: Denotes where the simulation will run once started. Set to
        'local' for localhost and 'lugano' for a dedicate machine on the Lugano viz cluster.
        :param context_id: The context ID if the experiment is declared in the collab portal
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
        self.__context_id = context_id
        self.__operation_mode = sim_operation_mode
        self.__creation_datetime = datetime.datetime.now(tz=timezone('Europe/Zurich'))
        self.__cle = None
        self.__state_machines_manager = StateMachineManager()
        self.__errors = 0  # We use that for monitoring

    @property
    def errors(self):
        """
        :return: the number of errors for the current simulation
        """
        return self.__errors

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
    def context_id(self):
        """
        The context ID of the navigation item of the collab portal used
        to create the experiment

        :return: The associated navigation item context_id
        """
        return self.__context_id

    @property
    def state(self):
        """
        Gets the state of the simulation.
        If the simulation CLE object is created then we do trust the CLE more than
        ourself and query it ! If the state is final (such as failed or stopped),
        then we don't bother with the CLE object.
        """
        if self.__cle is not None and self.__state not in ["stopped", "failed"]:
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
        # When the CLE changes the state without any intervention of the backend,
        # self.__state may not be properly updated. It results in false transition
        # errors. This is why we are calling the property at the beginning of this
        # method. Calling the property makes us refresh the local __state variable.
        reroute = reroutes.get(self.state)
        if reroute is not None:
            new_state = reroute.get(new_state, new_state)
        try:
            transitions = stateMachine[self.__state]
        except KeyError:
            raise InvalidStateTransitionException()
        if new_state not in transitions:
            raise InvalidStateTransitionException()
        try:
            transition = transitions[new_state]
            if transition is not None:
                # pylint: disable=not-callable
                transition(self)
            self.__state = new_state
        except:
            import sys
            exc = sys.exc_info()
            logger.exception(exc[1])
            self.__state = "halted"
            if 'halted' in transitions:
                try:
                    backup_transition = transitions['halted']
                    if backup_transition is not None:
                        # pylint: disable=not-callable
                        backup_transition(self)
                except Exception as e:
                    logger.exception(e)
                    msg = "Exception trying to set the simulation to state {0}: '{1}'. However, " \
                          "the cleanup command also failed with the error message '{2}'. Please " \
                          "contact your system administrator."\
                        .format(new_state, repr(exc[1]), repr(e))
                    self.__errors += 2
                    raise Exception(msg), None, exc[2]
            self.__errors += 1

            raise

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
    def state_machine_manager(self):
        """
        Gets the associated component managing the state machines of the simulation
        """
        return self.__state_machines_manager

    @property
    def state_machines(self):
        """
        Get state_machines contained in this simulation

        :return: The list of state machine instances
        """
        return self.__state_machines_manager.state_machines

    def get_state_machine(self, name):
        """
        Gets the state machine with the given name or none

        :param name: The state machine name
        :return: The state machine with the given name
        """
        return self.__state_machines_manager.get_state_machine(name)

    def get_state_machine_code(self, name):
        """
        Get state machine code

        :param name: name of the state machine
        :return: source code of the state machine (string) or False (bool) when \
        state machine is not found
        """
        sm = self.get_state_machine(name)
        if sm is not None and sm.sm_path is not None:
            with open(sm.sm_path, "r") as sm_file:
                return sm_file.read()

        return None

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

        with NamedTemporaryFile(prefix='sm_', suffix='.py', delete=False) as tmp:
            with tmp.file as sm_file:
                file_path = tmp.name
                sm_file.write(python_code)
        logger.info("Creating new file for state machine {0}: {1}".format(name, file_path))

        sm = self.get_state_machine(name)
        if sm is None:
            sm = StateMachineInstance(name)
            self.state_machines.append(sm)
        sm.sm_path = file_path

        if not sm.is_running:
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

        sm = self.get_state_machine(name)
        if sm is not None:
            self.state_machines.remove(sm)
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
        'contextID': fields.String(attribute='context_id')
    }
    required = [
        'state', 'simulationID', 'experimentConfiguration', 'gzserverHost', 'operationMode'
    ]


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
