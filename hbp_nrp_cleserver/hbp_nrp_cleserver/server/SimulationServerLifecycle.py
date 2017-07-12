# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains the simulation server implementation of the simulation lifecycle
"""

from hbp_nrp_commons.simulation_lifecycle import SimulationLifecycle
from hbp_nrp_cleserver.server import TOPIC_LIFECYCLE
from hbp_nrp_cle.tf_framework import TFException
from cle_ros_msgs.msg import CLEError
import threading
import logging
import sys

__author__ = 'Georg Hinkel'


logger = logging.getLogger(__name__)
# Sometimes previous handlers are cleared, make sure they are set
stdout_hdlr = logging.StreamHandler(sys.stdout)
stderr_hdlr = logging.StreamHandler(sys.stderr)
log_format = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
stdout_hdlr.setFormatter(logging.Formatter(log_format))
stderr_hdlr.setFormatter(logging.Formatter(log_format))
logger.handlers.append(stdout_hdlr)
logger.handlers.append(stderr_hdlr)


class SimulationServerLifecycle(SimulationLifecycle):
    """
    Implements the simulation server lifecycle of a simulation
    """

    def __init__(self, sim_id, cle, server, except_hook=None):
        self.stopped = lambda: None
        super(SimulationServerLifecycle, self).__init__(TOPIC_LIFECYCLE(sim_id))
        self.__start_thread = None
        self.__cle = cle
        self.__server = server
        self.__except_hook = except_hook or logger.exception
        self.__done_event = threading.Event()

    @property
    def done_event(self):
        """
        Gets the event that represents when the simulation is done

        :return: An event that will be set as soon as the lifecycle is done
        """
        return self.__done_event

    def __simulation(self):
        """
        Runs the Simulation and registers any exceptions
        """
        try:
            self.__cle.start()
        except TFException, e:
            self.__server.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, e.error_type,
                                        str(e), e.tf_name)
        except Exception, e:
            self.__server.publish_error("CLE", "General Error", str(e),
                                        severity=CLEError.SEVERITY_CRITICAL)
            self.failed()

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
            self.__cle.stop(forced=True)
        # pylint: disable=broad-except
        except Exception as e:
            self.__except_hook(e)
        if self.__start_thread is not None:
            self.__start_thread.join(60)
            if self.__start_thread.isAlive():
                logger.error(
                    "Error while stopping the simulation, "
                    "impossible to join the simulation thread"
                )

    def fail(self, state_change):
        """
        Reacts on failures in the simulation

        :param state_change: The state change according to the failure
        """
        self.__cle.stop(forced=True)
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
