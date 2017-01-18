"""
This module contains the simulation class
"""

__author__ = 'Georg Hinkel'

from hbp_nrp_backend.simulation_control import timezone
from hbp_nrp_excontrol.StateMachineManager import StateMachineManager
from hbp_nrp_excontrol.StateMachineInstance import StateMachineInstance
from hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle import \
    BackendSimulationLifecycle
from tempfile import NamedTemporaryFile
from flask_restful import fields
from flask_restful_swagger import swagger
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
                 reservation=None, sim_brain_processes=1, context_id=None, state='created'):
        """
        Creates a new simulation

        :param sim_id: The simulation id
        :param experiment_conf: The experiment configuration (Path to ExD configuration)
        :param environment_conf: The environment configuration (Path to environment configuration)
        :param owner: The name of the user owning the simulation
        :param sim_gzserver_host: Denotes where the simulation will run once started. Set to
        :param sim_brain_processes: Number of brain processes to use (overrides ExD configuration)
        'local' for localhost and 'lugano' for a dedicated machine on the Lugano viz cluster.
        :param context_id: The context ID if the experiment is declared in the collab portal
        :param state: The initial state (created by default)
        """
        self.__sim_id = sim_id
        self.__experiment_conf = experiment_conf
        self.__environment_conf = environment_conf
        self.__owner = owner
        self.__gzserver_host = sim_gzserver_host
        self.__context_id = context_id
        self.__reservation = reservation
        self.__brain_processes = sim_brain_processes
        self.__creation_datetime = datetime.datetime.now(tz=timezone)
        self.__cle = None
        self.__state_machines_manager = StateMachineManager()
        self.__kill_datetime = self.__creation_datetime + datetime.timedelta(minutes=30)
        self.__lifecycle = BackendSimulationLifecycle(self, state)

    @property
    def errors(self):
        """
        :return: the number of errors for the current simulation
        """
        return 1 if self.state in ['failed', 'halted'] else 0

    @property
    def kill_datetime(self):
        """
        Gets the time when the simulation should be killed
        """
        return self.__kill_datetime

    @kill_datetime.setter
    def kill_datetime(self, new_value):
        """
        Sets the time when the simulation should be killed

        :param new_value: The time when the simulation should be killed
        """
        self.__kill_datetime = new_value

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
    def context_id(self):
        """
        The context ID of the navigation item of the collab portal used
        to create the experiment

        :return: The associated navigation item context_id
        """
        return self.__context_id

    @property
    def lifecycle(self):
        """
        Gets the lifecycle of this simulation

        :return: The lifecycle instance
        """
        return self.__lifecycle

    @property
    def state(self):
        """
        Gets the state of the simulation

        :return: The state of the simulation as a string
        """
        return self.__lifecycle.state

    @state.setter
    def state(self, new_state):
        """
        Sets the simulation in a new state

        :param new_state: The new state
        """
        self.__lifecycle.accept_command(new_state)

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
        Only works when simulation is in state "paused"

        :param   name:        name of the state machine
        :type    name:        string
        :param   python_code: source code of the state machine
        :type    python_code: string
        :raise:  NameError, SyntaxError, AttributeError, ...
        """
        assert self.state == 'paused'

        with NamedTemporaryFile(prefix='sm_', suffix='.py', delete=False) as tmp:
            with tmp.file as sm_file:
                file_path = tmp.name
                sm_file.write(python_code)
        logger.info("Creating new file for state machine {0}: {1}".format(name, file_path))

        sm = self.get_state_machine(name)
        if sm is None:
            sm = StateMachineInstance(name, self.sim_id)
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

    def delete_all_state_machines(self):
        """
        Delete all state machines.
        """

        allsm = list(self.state_machines)
        for sm in allsm:
            self.delete_state_machine(sm.sm_id)

    @property
    def gzserver_host(self):
        """
        Gets the host/cluster where the gzserver is running or will run once the simulation is
        started.
        """
        return self.__gzserver_host

    @property
    def reservation(self):
        """
        Gets the cluster reservation that will be used to allocate a job running Gazebo.
        If the self.__reservation is None, the allocation request will not refer to any reservation.
        """
        return self.__reservation

    @property
    def brain_processes(self):
        """
        Gets the number of brain processes used for this simulation, overrides value in experiment
        configuration file
        """
        return self.__brain_processes

    resource_fields = {
        'state': fields.String,
        'simulationID': fields.Integer(attribute='sim_id'),
        'experimentConfiguration': fields.String(attribute='experiment_conf'),
        'environmentConfiguration': fields.String(attribute='environment_conf'),
        'owner': fields.String(attribute='owner'),
        'creationDate': fields.String(attribute=lambda x: x.creation_date),
        'gzserverHost': fields.String(attribute='gzserver_host'),
        'reservation': fields.String(attribute='reservation'),
        'contextID': fields.String(attribute='context_id'),
        'brainProcesses': fields.Integer(attribute='brain_processes')
    }
    required = [
        'state', 'simulationID', 'experimentConfiguration', 'gzserverHost'
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
