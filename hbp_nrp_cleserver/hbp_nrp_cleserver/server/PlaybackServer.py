# pylint: disable=too-many-lines
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
Playback ROS wrapper overriding the ROSCLEServer implementation for playback
"""

from hbp_nrp_cleserver.server.ROSCLEServer import ROSCLEServer
from hbp_nrp_cleserver.server.PlaybackServerLifecycle import PlaybackServerLifecycle

from hbp_nrp_cleserver.bibi_config.notificator import NotificatorHandler

import rospy
from cle_ros_msgs import srv
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from hbp_nrp_cleserver.server import SERVICE_SIM_RESET_ID, SERVICE_SIM_EXTEND_TIMEOUT_ID
import hbp_nrp_cle.tf_framework as tf_framework

import json
import time

import logging

logger = logging.getLogger(__name__)

# We use the logger hbp_nrp_cle.user_notifications in the CLE to log
# information that is useful to know for the user.
# In here, we forward any info message sent to this logger to the notificator
gazebo_logger = logging.getLogger('hbp_nrp_cle.user_notifications')
gazebo_logger.setLevel(logging.INFO)
notificator_handler = NotificatorHandler()


# disable pylint for protected _ROSCLEServer__<name> parent member access below
# pylint: disable=no-member, attribute-defined-outside-init
class PlaybackServer(ROSCLEServer):
    """
    Playback ROS server overriding the ROSCLEServer implementation for playback
    """

    def __init__(self, sim_id, timeout, gzserver, notificator):
        """
        Create the playback server

        :param sim_id: The simulation id
        :param timeout: The datetime when the simulation runs into a timeout
        :param gzserver: Gazebo simulator launch/control instance
        :param notificator: ROS state/error notificator interface
        """
        super(PlaybackServer, self).__init__(sim_id, timeout, gzserver, notificator)

        # simulation time from playback for frontend display
        self.__sim_clock = 0
        self.__sim_clock_subscriber = None

        # path to current active playback, accessible by Gazebo
        self.__playback_path = None

        # services to interact with playback plugin
        self.__service_configure = None
        self.__service_start = None
        self.__service_pause = None
        self.__service_stop = None
        self.__service_reset = None

    def prepare_simulation(self, playback_path, except_hook=None):
        """
        The CLE will be initialized within this method and ROS services for
        starting, pausing, stopping and resetting are setup here.

        :param playback_path: Recorded simulation playback path accessible by gzserver.
        :param except_hook: A handler method for critical exceptions
        """

        self.__playback_path = playback_path
        self._ROSCLEServer__lifecycle = PlaybackServerLifecycle(self._ROSCLEServer__simulation_id,
                                                                self,
                                                                except_hook)

        logger.info("Registering ROS Service handlers")

        # We have to use lambdas here (!) because otherwise we bind to the state which is in place
        # during the time we set the callback! I.e. we would bind directly to the initial state.
        # The x parameter is defined because of the architecture of rospy.
        # rospy is expecting to have handlers which takes two arguments (self and x). The
        # second one holds all the arguments sent through ROS (defined in the srv file).
        # Even when there is no input argument for the service, rospy requires this.

        # pylint: disable=unnecessary-lambda

        self._ROSCLEServer__service_reset = rospy.Service(
            SERVICE_SIM_RESET_ID(self._ROSCLEServer__simulation_id), srv.ResetSimulation,
            lambda x: self.reset_simulation(x)
        )

        self._ROSCLEServer__service_extend_timeout = rospy.Service(
            SERVICE_SIM_EXTEND_TIMEOUT_ID(self._ROSCLEServer__simulation_id), srv.ExtendTimeout,
            self._ROSCLEServer__extend_timeout
        )

        # use the playback clock values to populate state updates
        # pylint: disable=missing-docstring
        def clock_callback(t):
            self.__sim_clock = t.clock.secs
        self.__sim_clock_subscriber = rospy.Subscriber('/clock', Clock, clock_callback)

        tf_framework.TransferFunction.excepthook = self._ROSCLEServer__tf_except_hook
        self._ROSCLEServer__timer.start()

        # instantiate the service proxies
        base = '/gazebo/playback/%s'
        self.__service_configure = rospy.ServiceProxy(base % 'configure', srv.SimulationPlayback)
        self.__service_start = rospy.ServiceProxy(base % 'start', Trigger)
        self.__service_pause = rospy.ServiceProxy(base % 'pause', Trigger)
        self.__service_stop = rospy.ServiceProxy(base % 'stop', Trigger)
        self.__service_reset = rospy.ServiceProxy(base % 'reset', srv.SimulationPlayback)

        # configure the plugin path and check for failure
        resp = self.__service_configure(self.__playback_path)
        if not resp.success:
            raise Exception('Configuration of playback plugin failed: %s' % resp.message)

    def publish_state_update(self):
        """
        Publish the playback state and the remaining timeout
        """
        try:
            if self._ROSCLEServer__lifecycle is None:
                logger.warn("Trying to publish state even though no simulation is active")
                return
            message = {
                'state': str(self._ROSCLEServer__lifecycle.state),
                'timeout': self._ROSCLEServer__get_remaining(),
                'simulationTime': int(self.__sim_clock),
                'realTime': int(self.__sim_clock),
                'transferFunctionsElapsedTime': 0,
                'brainsimElapsedTime': 0,
                'robotsimElapsedTime': 0
            }
            logger.debug(json.dumps(message))
            self._ROSCLEServer__notificator.publish_state(json.dumps(message))
        # pylint: disable=broad-except
        except Exception as e:
            logger.exception(e)

    def shutdown(self):
        """
        Shutdown the playback
        """

        # the services are initialized in prepare_simulation, which is not
        # guaranteed to have occurred before shutdown is called
        if self.__sim_clock_subscriber is not None:
            logger.info("Shutting down reset service")
            self._ROSCLEServer__service_reset.shutdown()
            logger.info("Shutting down extend timeout service")
            self._ROSCLEServer__service_extend_timeout.shutdown()
            logger.info("Shutting down clock subscriber")
            self.__sim_clock_subscriber.unregister()
            time.sleep(1)

        # try to shutdown the service proxies, ignore failures and continue shutdown
        # pylint: disable=bare-except
        try:
            self.__service_configure.shutdown()
            self.__service_start.shutdown()
            self.__service_pause.shutdown()
            self.__service_stop.shutdown()
            self.__service_reset.shutdown()
        except:
            pass

        # items initialized in the constructor
        self._ROSCLEServer__lifecycle = None
        self._ROSCLEServer__timer.cancel_all()

    def process_lifecycle_command(self, command):
        """
        Issue a lifecycle state change as a playback plugin command

        :param command One of the supported commands [start, stop, pause]
        """

        # ensure we have configured the plugin before issuing commands
        if not self.__service_start or not self.__service_stop or not self.__service_pause:
            raise Exception('Playback server has not been configured, cannot issue commands!')

        # ensure the command is valid
        if command not in ['start', 'stop', 'pause']:
            raise ValueError('Invalid playback command: %s!' % command)

        # issue the service command
        # pylint: disable=bare-except
        try:
            if command == 'start':
                resp = self.__service_start()
            elif command == 'pause':
                resp = self.__service_pause()
            elif command == 'stop':
                resp = self.__service_stop()

            # check for playback command failure
            if not resp.success:
                raise Exception('Playback command failed: %s' % resp.message)

        # allow stop to fail during crash/shutdown
        except:
            if command != 'stop':
                raise

    def reset_simulation(self, request):
        """
        Reset the playback

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """

        # if uninitialized, we can't reset the path
        if not self.__playback_path:
            return False, 'Playback path is not configured, cannot reset!'

        # reset the internal status clock (probably happens anyway)
        self.__sim_clock = 0

        # perform the reset and (re-)configure
        resp = self.__service_reset(self.__playback_path)
        return resp.success, resp.message
