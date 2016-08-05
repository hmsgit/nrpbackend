"""
This module contains the simulation server implementation of the simulation lifecycle
"""

from hbp_nrp_commons.simulation_lifecycle import SimulationLifecycle
from hbp_nrp_cleserver.server import TOPIC_LIFECYCLE
from hbp_nrp_cleserver.server.DoubleTimer import DoubleTimer
from hbp_nrp_cle.tf_framework import TFException
import threading
import logging

__author__ = 'Georg Hinkel'


logger = logging.getLogger(__name__)


class SimulationServerLifecycle(SimulationLifecycle):
    """
    Implements the simulation server lifecycle of a simulation
    """

    STATUS_UPDATE_INTERVAL = 1.0

    def __init__(self, sim_id, cle, timeout, server):
        self.stopped = lambda: None
        super(SimulationServerLifecycle, self).__init__(TOPIC_LIFECYCLE(sim_id))
        self.__start_thread = None
        self.__cle = cle
        self.__timeout = timeout
        self.__server = server
        self.__double_timer = DoubleTimer(
            self.STATUS_UPDATE_INTERVAL,
            server.publish_state_update,
            self.__timeout,
            self.quit_by_timeout
        )
        self.__double_timer.start()
        self.start_timeout()
        self.__done_event = threading.Event()
        import sys
        logger.handlers.append(logging.StreamHandler(sys.stdout))
        logger.handlers.append(logging.StreamHandler(sys.stderr))

    @property
    def done_event(self):
        """
        Gets the event that represents when the simulation is done

        :return: An event that will be set as soon as the lifecycle is done
        """
        return self.__done_event

    def start_timeout(self):
        """
        Start the timeout on the current simulation
        """
        self.__double_timer.enable_second_callback()
        logger.info("Simulation will timeout in %f seconds", self.__timeout)

    def stop_timeout(self):
        """
        Stop the timeout
        """
        if self.__double_timer.is_expiring:
            self.__double_timer.disable_second_callback()
            logger.info("Timeout stopped")

    def quit_by_timeout(self):
        """
        Stops the simulation
        """
        self.stopped()
        logger.info("Force quitting the simulation")

    def remaining_time(self):
        """
        Gets the remaining time for the simulation
        """
        return self.__double_timer.remaining_time()

    def __simulation(self):
        """
        Runs the Simulation and registers any exceptions
        """
        try:
            self.__cle.start()
        except TFException, e:
            self.__server.publish_error("Transfer Function", e.error_type, str(e), e.tf_name)
        except Exception, e:
            self.__server.publish_error("CLE", "General Error", str(e))
            raise

    def shutdown(self, shutdown_event):
        """
        Shuts down this instance of the simulation lifecycle

        :param shutdown_event: The event that caused the shutdown
        """
        self.__done_event.set()
        super(SimulationServerLifecycle, self).shutdown(shutdown_event)

    def initialize(self, state_change):
        """
        Initializes the simulation

        :param state_change: The state change that caused the simulation to initialize
        """
        if not self.__cle.is_initialized:
            self.__cle.initialize()

    def start(self, state_change):
        """
        Starts the simulation

        :param state_change: The state change that caused the simulation to start
        """
        self.__start_thread = threading.Thread(target=self.__simulation)
        self.__start_thread.setDaemon(True)
        self.__start_thread.start()

    def stop(self, state_change):
        """
        Stops the simulation and releases required resources

        :param state_change: The state change that caused the simulation to stop
        """
        try:
            self.stop_timeout()
            self.__double_timer.cancel_all()
            self.__cle.stop()
            if self.__start_thread is not None:
                self.__start_thread.join(60)
                if self.__start_thread.isAlive():
                    logger.error(
                        "Error while stopping the simulation, "
                        "impossible to join the simulation thread"
                    )
        finally:
            self.__double_timer.join()

    def fail(self, state_change):
        """
        Reacts on failures in the simulation

        :param state_change: The state change according to the failure
        """
        self.stop_timeout()
        self.__double_timer.cancel_all()
        self.__cle.stop()
        self.__server.publish_state_update()

    def pause(self, state_change):
        """
        Pauses the simulation

        :param state_change: The state change that caused the pause request
        """
        self.__cle.stop()

    def reset(self, state_change):
        """
        Resets the simulation

        :param state_change:
        :return:
        """
        try:
            self.__server.start_fetching_gazebo_logs()
            self.__cle.stop()
            self.__cle.reset()
        finally:
            self.__server.stop_fetching_gazebo_logs()
