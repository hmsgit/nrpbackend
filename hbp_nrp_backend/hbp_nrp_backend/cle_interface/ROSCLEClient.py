"""
Takes care of making the appropriate ROS call(s) to control a simulation.
On the other side of ROS, the calls are handled by ROSCLEServer.py
"""

import logging
import rospy
from std_srvs.srv import Empty
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of the CLE (this) repository.
from cle_ros_msgs import srv
from hbp_nrp_backend.cle_interface import SERVICE_SIM_START_ID, SERVICE_SIM_PAUSE_ID, \
    SERVICE_SIM_STOP_ID, SERVICE_SIM_RESET_ID, SERVICE_SIM_STATE_ID, \
    SERVICE_GET_TRANSFER_FUNCTIONS, SERVICE_SET_TRANSFER_FUNCTION, \
    SERVICE_DELETE_TRANSFER_FUNCTION  # duplicated from CLE.__init__
from hbp_nrp_backend.cle_interface.ROSCLEState import ROSCLEState  # duplicated from CLE

__author__ = "Lorenzo Vannucci, Daniel Peppicelli"
logger = logging.getLogger(__name__)

# pylint: disable=W0710


class ROSCLEClientException(Exception):
    """
    Exception within the CLE client
    """
    pass


class ROSCLEClient(object):
    """
    Client around the ROS controlled Closed Loop Engine.
    """
    ROS_SERVICE_TIMEOUT = 180

    def __init__(self, sim_id):
        """
        Create the wrapper client
        :param sim_id: The simulation id
        """

        self.__valid = True
        self.__invalid_reason = ""

        # Creates service proxies
        self.__cle_start = self.__init_ros_service(SERVICE_SIM_START_ID(sim_id), Empty)
        self.__cle_pause = self.__init_ros_service(SERVICE_SIM_PAUSE_ID(sim_id), Empty)
        self.__cle_stop = self.__init_ros_service(SERVICE_SIM_STOP_ID(sim_id), Empty)
        self.__cle_reset = self.__init_ros_service(SERVICE_SIM_RESET_ID(sim_id), Empty)
        self.__cle_state = self.__init_ros_service(SERVICE_SIM_STATE_ID(sim_id),
                                                   srv.GetSimulationState)
        self.__cle_get_transfer_functions = \
            self.__init_ros_service(SERVICE_GET_TRANSFER_FUNCTIONS(sim_id),
                                    srv.GetTransferFunctions)

        self.__cle_set_transfer_function = \
            self.__init_ros_service(SERVICE_SET_TRANSFER_FUNCTION(sim_id),
                                    srv.SetTransferFunction)

        self.__cle_delete_transfer_function = \
            self.__init_ros_service(SERVICE_DELETE_TRANSFER_FUNCTION(sim_id),
                                    srv.DeleteTransferFunction)

    def __init_ros_service(self, service_name, service_class):
        """
        Initialize a ROS service proxy

        @param: service_name: The name of the service
        @param: service_class: The class (.srv) of the service. This class
                               is generated from the ROS catkin package .srv files.
        """
        handler = None
        if self.__valid:
            try:
                logger.info("Connecting to ROS service " + service_name)
                handler = rospy.ServiceProxy(service_name, service_class)
                handler.wait_for_service(timeout=self.ROS_SERVICE_TIMEOUT)
            except rospy.ROSException:
                # According to the documentation, only a timeout will raise a generic
                # 'ROSException'.
                # http://docs.ros.org/api/rospy/html/rospy-module.html#wait_for_service
                message = "Timeout while connecting to the CLE (waiting on %s)." % \
                          (service_name, )
                logger.error(message)
                # Difficult to understand why pylint considers ROSCLEClientException as a
                # non standard exception. If you have an idea, please correct it!
                # pylint: disable=nonstandard-exception
                raise ROSCLEClientException(message)
        return handler

    def __call_service(self, service):
        """
        Generic call handler for all the ROS service proxies that are based on the Empty class.
        The call handler will mask exceptions from ROS and will discard the client if there is
        something wrong going on.
        """
        if self.__valid:
            try:
                service()
            except rospy.ServiceException:
                self.__valid = False
                self.__invalid_reason = "a previous communication error"
                # pylint: disable=nonstandard-exception
                raise ROSCLEClientException(
                    "Impossible to communicate with the CLE, discarding the client."
                )
        else:
            # pylint: disable=nonstandard-exception
            raise ROSCLEClientException(
                "Client has been discarded due to %s." % (self.__invalid_reason, )
            )

    def start(self):
        """
        Start the simulation.
        """
        self.__call_service(self.__cle_start)

    def pause(self):
        """
        Pause the simulation.
        """
        self.__call_service(self.__cle_pause)

    def stop(self):
        """
        Stop the simulation.
        """
        self.__call_service(self.__cle_stop)
        # Once the stop handler has been called, we do not expect it to answer anymore
        self.__valid = False
        self.__invalid_reason = "a previous stop request (triggering automatic disconnection)"

    def reset(self):
        """
        Reset the simulation.
        """
        self.__call_service(self.__cle_reset)

    def get_simulation_state(self):
        """
        Get the simulation state.
        """
        # By default, we assume an experiment is in the stop state
        # (whether it is really stop or if something bad did happen.)
        state = ROSCLEState.STOPPED
        if self.__valid:
            try:
                state = str(self.__cle_state().state)
            except rospy.ServiceException as e:
                logger.error(
                    "Error while trying to retrieve simulation state: %s. "
                    "Returning stopped as state.",
                    str(e)
                )
        else:
            logger.warn(
                "Trying to retrieve the state of a simulation from an invalid client "
                "(invalid due to %s).",
                self.__invalid_reason
            )
        return state

    def get_simulation_transfer_functions(self):
        """
        Get the simulation transfer functions.

        :returns: An array of strings containing the source code of the transfer
                  functions.
        """
        result = []
        if self.__valid:
            try:
                response = self.__cle_get_transfer_functions()
                result = response.transfer_functions
            except rospy.ServiceException as e:
                logger.error(
                    "Error while trying to retrieve simulation transfer functions: %s. "
                    "Returning an empty transfer function array",
                    str(e)
                )
        else:
            logger.warn(
                "Trying to retrieve the transfer functions of a simulation from "
                "an invalid client (invalid due to %s).",
                self.__invalid_reason
            )
        return result

    def delete_simulation_transfer_function(
            self,
            transfer_function_name):
        """
        Delete the given transfer function

        :param transfer_function_name: Name of the transfer function to delete
        :return: True if the call to ROS is successful, False otherwise
        """
        result = False
        if self.__valid:
            try:
                response = self.__cle_delete_transfer_function(
                    transfer_function_name
                )
                result = response.success
            except rospy.ServiceException as e:
                logger.error(
                    "Error while deleting transfer function %s: %s.",
                    transfer_function_name,
                    str(e)
                )
        else:
            logger.warn(
                "Trying to delete transfer functions %s of a simulation from "
                "an invalid client (invalid due to %s).",
                transfer_function_name,
                self.__invalid_reason
            )
        return result

    def set_simulation_transfer_function(
            self,
            transfer_function_name,
            transfer_function_source
    ):
        """
        Set the simulation transfer function's source code.

        :param transfer_function_name: Name of the transfer function to modifiy or create
        :param transfer_function_source: Source code of the transfer function
        :returns: "" if the call to ROS is successful,
                     a string containing an error message otherwise
        """
        result = ""
        if self.__valid:
            try:
                response = self.__cle_set_transfer_function(
                    transfer_function_name,
                    transfer_function_source
                )
                result = response.error_message
            except rospy.ServiceException as e:
                logger.error(
                    "Error while setting the code of transfer function (%s, %s)"
                    "from the simulation: %s.",
                    transfer_function_name,
                    transfer_function_source,
                    str(e)
                )
        else:
            logger.warn(
                "Trying to set transfer functions %s of a simulation from "
                "an invalid client (invalid due to %s).",
                transfer_function_name,
                self.__invalid_reason
            )
        return result
