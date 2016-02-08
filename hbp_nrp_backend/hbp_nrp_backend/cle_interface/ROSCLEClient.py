"""
Takes care of making the appropriate ROS call(s) to control a simulation.
On the other side of ROS, the calls are handled by ROSCLEServer.py
"""

import logging
import rospy
import timeout_decorator
from std_srvs.srv import Empty
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackages repository.
from cle_ros_msgs import srv
from hbp_nrp_backend.cle_interface import SERVICE_SIM_START_ID, SERVICE_SIM_PAUSE_ID, \
    SERVICE_SIM_STOP_ID, SERVICE_SIM_RESET_ID, SERVICE_SIM_STATE_ID, \
    SERVICE_GET_TRANSFER_FUNCTIONS, SERVICE_SET_TRANSFER_FUNCTION, \
    SERVICE_DELETE_TRANSFER_FUNCTION, SERVICE_SET_BRAIN, SERVICE_GET_BRAIN
    # duplicated from CLE.__init__
from cle_ros_msgs.srv import ResetSimulation
from hbp_nrp_backend.cle_interface.ROSCLEState import ROSCLEState  # duplicated from CLE

__author__ = "Lorenzo Vannucci, Daniel Peppicelli"
logger = logging.getLogger(__name__)


def fallback_retval(val):
    """
    This function is intended to be used as a decorator for the methods calling CLE services.
    It intercepts a ROSCLEClientException during the execution of a ROSServiceWrapper and returns
    the desired value instead.
    :param val: the fallback return value for the decorated method
    """
    def decorator(func):
        """
        The decorator function returned once the wanted fallback return value is set
        :param func: the function to decorate.
        """
        def f(*args, **kwargs):
            """
            The function actually executed instead of the decorated one.
            """
            try:
                return func(*args, **kwargs)
            except ROSCLEClientException:
                return val
        return f
    return decorator


class ROSCLEClientException(Exception):
    """
    Exception within the CLE client
    """
    pass


class ROSCLEServiceWrapper(object):
    """
    Wraps the behaviour of a standard ROS service, throwing a detailed ROSCLEClientException
    in case of invalid client or ROS exceptions.
    """
    ROS_SERVICE_TIMEOUT = 180

    def __init__(self, service_name, service_class, ros_cle_client, invalidate_on_failure=False):
        """
        :param service_name: the name of the ROS service.
        :param service_class: the class of the ROS service parameter.
        :param ros_cle_client: the ROSCLEClient instance creating the wrapper.
        :param invalidate_on_failure (default=False): a boolean value deciding whether the
            ROSCLEClient should be invalidated in case of failure (True) or not (False).
        """
        self.__handler = None
        self.__ros_cle_client = ros_cle_client
        self.__invalidate_on_failure = invalidate_on_failure
        try:
            logger.info("Connecting to ROS service " + service_name)
            self.__handler = rospy.ServiceProxy(service_name, service_class)
            self.__handler.wait_for_service(timeout=self.ROS_SERVICE_TIMEOUT)
        except rospy.ROSException:
            # According to the documentation, only a timeout will raise a generic
            # 'ROSException'.
            # http://docs.ros.org/api/rospy/html/rospy-module.html#wait_for_service
            message = "Timeout while connecting to the CLE (waiting on %s)." % \
                      (service_name, )
            logger.error(message)
            raise ROSCLEClientException(message)

    @timeout_decorator.timeout(ROS_SERVICE_TIMEOUT)
    def __call__(self, *args, **kwargs):
        if self.__ros_cle_client.valid:
            try:
                return self.__handler(*args, **kwargs)
            except (rospy.ServiceException, rospy.exceptions.ROSInterruptException) as e:
                if self.invalidate_on_failure:
                    self.__ros_cle_client.valid = False
                    self.__ros_cle_client.invalid_reason = "a previous communication error"
                message = "Error executing service \"%s\", unable to communicate with the CLE.\n" \
                      "Error: %s" % (self.__handler.resolved_name, str(e), )
                logger.error(message)
                raise ROSCLEClientException(message)
        else:
            message = "Action \"%s\" can't be peformed on an invalid client (reason: %s)." % \
                  (self.__handler.resolved_name, self.__ros_cle_client.invalid_reason)
            logger.error(message)
            raise ROSCLEClientException(message)

    @property
    def handler(self):
        """
        Property getter for the __handler attribute
        """

        return self.__handler

    @property
    def ros_cle_client(self):
        """
        Property getter for the __ros_cle_client attribute
        """

        return self.__ros_cle_client

    @property
    def invalidate_on_failure(self):
        """
        Property getter for the __invalidate_on_failure attribute
        """

        return self.__invalidate_on_failure


class ROSCLEClient(object):
    """
    Client around the ROS controlled Closed Loop Engine.
    """

    def __init__(self, sim_id):
        """
        Create the wrapper client
        :param sim_id: The simulation id
        """
        self.valid = True
        self.invalid_reason = ""

        # Creates service proxies
        self.__cle_start = ROSCLEServiceWrapper(SERVICE_SIM_START_ID(sim_id), Empty, self,
                                                invalidate_on_failure=True)
        self.__cle_pause = ROSCLEServiceWrapper(SERVICE_SIM_PAUSE_ID(sim_id), Empty, self,
                                                invalidate_on_failure=True)
        self.__cle_stop = ROSCLEServiceWrapper(SERVICE_SIM_STOP_ID(sim_id), Empty, self)
        self.__cle_reset = ROSCLEServiceWrapper(SERVICE_SIM_RESET_ID(sim_id), ResetSimulation,
                                                self, invalidate_on_failure=True)
        self.__cle_state = ROSCLEServiceWrapper(
            SERVICE_SIM_STATE_ID(sim_id), srv.GetSimulationState, self)
        self.__cle_get_transfer_functions = ROSCLEServiceWrapper(
            SERVICE_GET_TRANSFER_FUNCTIONS(sim_id), srv.GetTransferFunctions, self)

        self.__cle_set_transfer_function = ROSCLEServiceWrapper(
            SERVICE_SET_TRANSFER_FUNCTION(sim_id), srv.SetTransferFunction, self)

        self.__cle_delete_transfer_function = ROSCLEServiceWrapper(
            SERVICE_DELETE_TRANSFER_FUNCTION(sim_id), srv.DeleteTransferFunction, self)

        self.__cle_get_brain = ROSCLEServiceWrapper(SERVICE_GET_BRAIN(sim_id), srv.GetBrain, self)
        self.__cle_set_brain = ROSCLEServiceWrapper(SERVICE_SET_BRAIN(sim_id), srv.SetBrain, self)

    def start(self):
        """
        Start the simulation.
        """
        self.__cle_start()

    def pause(self):
        """
        Pause the simulation.
        """
        self.__cle_pause()

    def stop(self):
        """
        Stop the simulation.
        """
        self.__cle_stop()
        self.valid = False
        self.invalid_reason = "a previous stop request (triggering automatic disconnection)"

    def reset(self, reset_robot_pose=False, full_reset=False):
        """
        Reset the simulation.
        :param reset_robot_pose: True if we want to reset the robot initial pose, False otherwise.
        :param full_reset: True if we want to perform a full reset, False otherwise.
        """
        # TODO: Uniform response from ROS CLE services so that this could be done directly
        # in the wrapper class
        resp = self.__cle_reset(reset_robot_pose, full_reset)
        if not resp.success:
            raise ROSCLEClientException(resp.error_message)

    # By default, we assume an experiment is in the stop state
    # (whether it is really stop or if something bad did happen.)
    @fallback_retval(ROSCLEState.STOPPED)
    def get_simulation_state(self):
        """
        Get the simulation state.
        """
        return str(self.__cle_state().state)

    @fallback_retval({})
    def get_simulation_brain(self):
        """
        Get the brain of the running simulation

        :return: dict with brain_data, brain_type, data_type
        """
        return self.__cle_get_brain()

    def set_simulation_brain(self, brain_type, data, data_type, brain_populations):
        """
        Set the brain of the running simulation (will pause the simulation)

        :param data: brain data, data type defined in data_type
        :param brain_type: currently "py" or "h5"
        :param data_type: data type ("text" or "base64")
        :param brain_populations: A dictionary indexed by population names and
        containing neuron indices. Neuron indices could be defined by individual integers,
        lists of integers or python slices. Python slices are defined by a
        dictionary containing the 'from', 'to' and 'step' values.
        :return: response of the cle
        """
        return self.__cle_set_brain(brain_type, data, data_type, brain_populations)

    @fallback_retval([])
    def get_simulation_transfer_functions(self):
        """
        Get the simulation transfer functions.

        :returns: An array of strings containing the source code of the transfer
                  functions.
        """
        return self.__cle_get_transfer_functions().transfer_functions

    @fallback_retval(False)
    def delete_simulation_transfer_function(self, transfer_function_name):
        """
        Delete the given transfer function

        :param transfer_function_name: Name of the transfer function to delete
        :return: True if the call to ROS is successful, False otherwise
        """
        return self.__cle_delete_transfer_function(transfer_function_name).success

    def set_simulation_transfer_function(self, transfer_function_name, transfer_function_source):
        """
        Set the simulation transfer function's source code.

        :param transfer_function_name: Name of the transfer function to modify or create
        :param transfer_function_source: Source code of the transfer function
        :returns: "" if the call to ROS is successful,
                     a string containing an error message otherwise
        """
        return self.__cle_set_transfer_function(transfer_function_name, transfer_function_source) \
                   .error_message
