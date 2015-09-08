"""
This module contains the class which encapsulates the dynamically loaded smach state machines
 and provides the routines for their execution in parallel.
"""

from hbp_nrp_backend.experiment_control import MAX_TIMEOUT, TimeoutException, \
    ConcurrentExecutionException
from numbers import Number
from types import ModuleType
import threading
import logging

__author__ = 'Sebastian Krach'

logger = logging.getLogger(__name__)


class ExperimentStateMachineInstance(object):
    """
    The class that encapsulates experiment controlling state machines and manages their execution
    in separate threads
    """

    def __init__(self, sm_id, sm_source=None):
        """
        Creates a new uninitialized ExperimentStateMachineInstance

        :param sm_id: the unique identifier of the state machine
        :param source: the python source of the state machine. Optional, as it can be set later.
        """

        self.__sm_id = sm_id

        self.__sm_module = None

        if sm_source is not None:
            self.__sm_module = self.__try_compile_module(sm_id, sm_source)
            if self.__sm_module is None:
                raise AttributeError("The passed source code does not describe a valid SMACH state"
                                     "machine")
        self.__sm_source = sm_source

        self.__sm_instance = None
        self.__machine_thread = None
        self.__result = None
        self.__finished_event = threading.Event()

    @property
    def sm_id(self):
        """
        Gets the unique state machine identifier specified upon creation.

        :return: The unique state machine identifier.
        """

        return self.__sm_id

    @property
    def sm_source(self):
        """
        Gets the python source code for the encapsulated SMACH state machine.

        :return: The python source code of the SMACH state machine
        """
        return self.__sm_source

    @sm_source.setter
    def sm_source(self, value):
        """
        Sets the python source of the encapsulated SMACH state machine if the passed code is valid
        and a compatible state machine could be initialized from it. Setting the source code is only
        possible whenever the state machine is not executing.

        Specifying a new state machine source leads to an uninitialization of this state machine
        instance iff the passed source code is valid. It is thereafter required to call
        initialize_sm() upon the newly loaded state machine to initialize it.

        :param value: The new state machine source code.
        """

        if self.is_running:
            raise ConcurrentExecutionException(
                "The state machine is already running in a distinct thread. Stop execution first.")

        if self.sm_source != value:
            new_module = self.__try_compile_module(self.sm_id, value)
            if new_module is None:
                raise AttributeError("The passed source code does not describe a valid SMACH state"
                                     "machine")
            self.__sm_module = new_module
            self.__sm_source = value
            self.__sm_instance = None
            self.__machine_thread = None

    @property
    def sm_module(self):
        """
        Gets the python module instantiated from the script describing the SMACH state machine.
        Access allows to interact with modules loaded by state machine as state machine module is
        not loaded into sys.modules.

        :return: The python module, iff the a proper state machine source code has been set. None,
                 otherwise.
        """

        return self.__sm_module

    @property
    def sm_instance(self):
        """
        Gets the instantiated encapsulated state machine if this instance is properly initialized
        and the script file could be loaded successfully.

        :return: the instantiated state machine, else None
        """

        return self.__sm_instance

    @property
    def is_running(self):
        """
        Gets for the encapsulated state machine whether it is executing or not.

        :return: True if the state machine is executing, False otherwise.
        """

        return self.__machine_thread is not None and \
               self.__machine_thread.is_alive() and \
               self.__sm_instance.sm.is_running()

    @property
    def result(self):
        """
        If the execution of the encapsulated state machine has finished it returns the final
        outcome.

        :return: the final outcome, None otherwise
        """

        if self.is_running:
            return None
        else:
            return self.__result

    def start_execution(self):
        """
        Starts the execution of the state machine in a separat thread.
        """

        if self.__machine_thread is None or not self.__machine_thread.is_alive():
            if self.__sm_instance is not None:
                self.__machine_thread = threading.Thread(target=self.__run,
                                                         args=[self.__sm_instance])
                self.__machine_thread.daemon = True

                self.__finished_event.clear()
                self.__machine_thread.start()
            else:
                raise AttributeError("No SMACH state machine specified")
        else:
            raise ConcurrentExecutionException(
                "The state machine is already running in a distinct thread. Stop execution first.")

    def initialize_sm(self):
        """
        Loads the specified script file and initializes the encapsulated state machine.
        """

        if self.__sm_module is None:
            raise AttributeError(
                "The state machine cannot be initialized as there is no source code specified to "
                "initialize the state machine from.")
        if self.is_running:
            raise ConcurrentExecutionException("The state machine is already running")
        self.__sm_instance = self.__instantiate_state_machine(self.__sm_module)
        self.__sm_instance.sm.register_termination_cb(self.__termination_cb)

    def request_termination(self):
        """
        Requests termination of the encapsulated state machine. As the termination happens
        asynchronously this method will return immediately after setting the request flag.
        If the state machine has already finished execution the method simply returns.
        """

        if self.__machine_thread is None:
            raise AttributeError("No state machine has been running. Cannot request termination")
        if self.is_running:
            self.__sm_instance.sm.request_preempt()

    def wait_termination(self, timeout=MAX_TIMEOUT):
        """
        Blocks until the encapsulated state machine has finished execution and a result is
        available

        :param timeout: Maximum waiting time in seconds
        """

        if self.__machine_thread is None:
            raise AttributeError("No state machine has been running. Cannot wait for termination")

        if not isinstance(timeout, Number) or timeout > MAX_TIMEOUT:
            logger.warn("Timeout not a valid number or greater than allowed maximum. Set to "
                        "maximum.")
            timeout = MAX_TIMEOUT

        if self.__finished_event.wait(timeout) is not True:
            raise TimeoutException("The state machine did not terminate during the specified "
                                   "period of time")

        self.__machine_thread.join(timeout)
        if self.__machine_thread.is_alive():
            raise TimeoutException("The thread executing the state machine did not terminate "
                                   "during the specified period of time")

        return self.__result

    def __termination_cb(self, userdata, term_state, outcome):
        """
        Callback method, which is invoked when the smach state machine terminates. It is
        supposed to set the event that blocks waiting threads and to accept the outcome
        of the terminated state machine

        :param userdata: The userdata which is passed from state to state through the smach state
        machine.
        :param term_states: The last state machine state which produced the final outcome.
        :param outcome: The outcome of the state machine.
        """

        self.__result = outcome
        self.__finished_event.set()

    @staticmethod
    def __run(sm):
        """
        Method, that is called on spawned thread and starts the execution of the smach state
        machine

        :param sm: the state machine to execute
        """

        if sm is not None:
            sm.execute()

    @staticmethod
    def __try_compile_module(sm_id, sm_module_source):
        """
        Private method that tries to instatiate the python module from the passed source and checks
        whether the module provides the required create_state_machine() method.

        :param sm_module_source: The source code of the python module to instantiate
        :return: The module, if the module provides the required method. None, otherwise
        """
        sm_module = ModuleType(str(sm_id))
        # pylint: disable=exec-used
        exec (sm_module_source, sm_module.__dict__)
        # pylint: enable=exec-used
        if hasattr(sm_module, "create_state_machine"):
            return sm_module
        else:
            return None

    @staticmethod
    def __instantiate_state_machine(sm_module):
        """
        Tries to load the provided python script and to instantiate the contained state
        machine

        :param sm_module_source: the python script source of the module containing the state machine
        :return: the instantiated state machine
        """

        return sm_module.create_state_machine()
