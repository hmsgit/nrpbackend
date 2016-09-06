"""
Helper class that takes care of making the appropriate ROS call(s) to
start a simulation.
"""

import rospy
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the root folder of the GazeboRosPackage repository.
from cle_ros_msgs import srv
from hbp_nrp_backend.cle_interface import SERVICE_CREATE_NEW_SIMULATION

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli"


class ROSCLESimulationFactoryClient(object):

    """
    Helper class for the simulation factory ROS service started with class
    ROSCLESimulationFactory.
    """

    def __init__(self):
        """
        Create the client. If the service is not available after 10 seconds, a ROSException
        will be raised.
        """

        self.__create_new_simulation_service = rospy.ServiceProxy(
            SERVICE_CREATE_NEW_SIMULATION, srv.CreateNewSimulation
        )
        self.__create_new_simulation_service.wait_for_service(timeout=10)

    def create_new_simulation(self, environment_file, experiment_conf, gzserver_host,
                              brain_processes, sim_id):
        """
        Start the simulation.
        """
        response = self.__create_new_simulation_service(environment_file, experiment_conf,
                                                        gzserver_host, brain_processes, sim_id)
        success, status_message = response.success, response.status_message
        if not success:
            raise SimulationStartingFailureException(status_message)


class SimulationStartingFailureException(Exception):
    """
    Exception raised when the create_new_simulation fails.
    """
    pass
