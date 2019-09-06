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
This module contains the abstract base class for a simulation assembly
that uses Gazebo for world simulation
"""

import logging
import netifaces
import os
import subprocess
import rospy
import rosnode

logger = logging.getLogger(__name__)

from hbp_nrp_cle.cle import config
from hbp_nrp_cleserver.server.SimulationAssembly import SimulationAssembly
from hbp_nrp_cleserver.server.ROSLaunch import ROSLaunch
from hbp_nrp_cleserver.server.LocalGazebo import LocalGazeboBridgeInstance, \
    LocalGazeboServerInstance
from hbp_nrp_cleserver.server.LuganoVizClusterGazebo import LuganoVizClusterGazebo, XvfbXvnError
from hbp_nrp_cleserver.server.GazeboSimulationRecorder import GazeboSimulationRecorder
from hbp_nrp_cle.robotsim.RobotManager import RobotManager

from hbp_nrp_commons.workspace.SimUtil import SimUtil


class GazeboSimulationAssembly(SimulationAssembly):     # pragma: no cover
    """
    The abstract base class for a simulation assembly that uses Gazebo for world simulation
    """

    def __init__(self, sim_config):
        """
        Creates a new simulation assembly to simulate an experiment using the CLE and Gazebo
        :param sim_config: config of the simulation to be managed
        """
        super(GazeboSimulationAssembly, self).__init__(sim_config)
        self.robotManager = RobotManager()

        if self.sim_config.gzserver_host == 'local':
            self.gzserver = LocalGazeboServerInstance()
        elif self.sim_config.gzserver_host == 'lugano':
            self.gzserver = LuganoVizClusterGazebo(self.sim_config.timeout.tzinfo
                                                   if self.sim_config.timeout is not None
                                                   else None, self.sim_config.reservation)
        else:
            raise Exception("The gzserver location '{0}' is not supported.",
                            self.sim_config.gzserver_host)

        self.gzweb = None
        self._initial_ros_params = None
        self._initial_ros_nodes = None
        self.ros_launcher = None
        self.gazebo_recorder = None

    def _start_gazebo(self, extra_models):
        """
        Configures and starts the Gazebo simulator and backend services

        :param rng_seed: RNG seed to spawn Gazebo with
        :param playback_path: A path to playback information
        :param extra_models: An additional models path or None
        :param world_file: The world file that should be loaded by Gazebo
        """
        self._initial_ros_params = rospy.get_param_names()
        self._initial_ros_nodes = rosnode.get_node_names()

        # Gazebo configuration and launch
        self._notify("Starting Gazebo robotic simulator")
        ifaddress = netifaces.ifaddresses(config.config.get('network', 'main-interface'))
        local_ip = ifaddress[netifaces.AF_INET][0]['addr']
        ros_master_uri = os.environ.get("ROS_MASTER_URI").replace('localhost', local_ip)

        self.gzserver.gazebo_died_callback = self._handle_gazebo_shutdown

        # experiment specific gzserver command line arguments
        gzserver_args = '--seed {rng_seed} -e {engine} {world_file}'.format(
            rng_seed=self.rng_seed, engine=self.sim_config.physics_engine,
            world_file=self.sim_config.world_model.resource_path.abs_path)

        # If playback is specified, load the first log/world file in the recording at Gazebo launch
        if self.sim_config.playback_path:
            gzserver_args += ' --play {path}'.format(
                path=os.path.join(self.sim_config.playback_path, 'gzserver/1.log'))
        else:
            # optional roslaunch support prior to Gazebo launch for non-playback simulations
            if self.sim_config.ros_launch_abs_path is not None:
                if self.sim_config.gzserver_host != 'local':
                    raise Exception('roslaunch is currently only supported on local installs.')

                self._notify("Launching experiment ROS nodes and configuring parameters")
                self.ros_launcher = ROSLaunch(self.sim_config.ros_launch_abs_path)

        try:
            logger.info("gzserver arguments: " + gzserver_args)
            self.gzserver.start(ros_master_uri, extra_models, gzserver_args)
        except XvfbXvnError as exception:
            logger.error(exception)
            error = "Recoverable error occurred. Please try again. Reason: {0}".format(exception)
            raise Exception(error)

        self._notify("Connecting to Gazebo robotic simulator")
        self.robotManager.init_scene_handler()

        self._notify("Connecting to Gazebo simulation recorder")
        self.gazebo_recorder = GazeboSimulationRecorder(self.sim_config.sim_id)

        self._notify("Starting Gazebo web client")
        os.environ['GAZEBO_MASTER_URI'] = self.gzserver.gazebo_master_uri

        self.__set_env_for_gzbridge()

        # We do not know here in which state the previous user did let us gzweb.
        self.gzweb = LocalGazeboBridgeInstance()
        self.gzweb.restart()

    # pylint: disable=missing-docstring
    def __set_env_for_gzbridge(self):
        os.environ['GZBRIDGE_POSE_FILTER_DELTA_TRANSLATION'] = self.sim_config.gzbridge_setting(
            'pose_update_delta_translation', 1.e-5)
        os.environ['GZBRIDGE_POSE_FILTER_DELTA_ROTATION'] = self.sim_config.gzbridge_setting(
            'pose_update_delta_rotation', 1.e-4)
        os.environ['GZBRIDGE_UPDATE_EARLY_THRESHOLD'] = self.sim_config.gzbridge_setting(
            'pose_update_early_threshold', 0.02)

    def _handle_gazebo_shutdown(self):  # pragma: no cover
        """
        Handles the case that gazebo died unexpectedly
        """
        logger.exception("Gazebo died unexpectedly")
        # Avoid further notice
        self.gzserver.gazebo_died_callback = None
        # in case the simulation is still being started, we abort the initialization
        self._abort_initialization = "Gazebo died unexpectedly"

    def shutdown(self):
        """
        Shutdown CLE
        """
        # Once we do reach this point, the simulation is stopped
        # and we can clean after ourselves.
        # pylint: disable=broad-except, too-many-branches,too-many-statements

        # Clean up gazebo after ourselves
        number_of_subtasks = 4
        if self.sim_config.ros_launch_abs_path is not None:
            number_of_subtasks += 1
        if self.sim_config.ext_robot_controller is not None:
            number_of_subtasks += 1

        try:

            # Check if notifications to clients are currently working
            try:
                self.ros_notificator.start_task("Stopping simulation",
                                                "Shutting down simulation recorder",
                                                number_of_subtasks=number_of_subtasks,
                                                block_ui=False)
                notifications = True
            except Exception, e:
                logger.error("Could not send notifications")
                logger.exception(e)
                notifications = False

            # Call the recorder plugin to shutdown before shutting down Gazebo
            if self.gazebo_recorder is not None:
                try:
                    self.gazebo_recorder.shutdown()
                except Exception, e:
                    logger.warning("Gazebo recorder could not be shutdown successfully")
                    logger.exception(e)

            self._shutdown(notifications)

            # Shutdown gzweb before shutting down Gazebo
            if self.gzweb is not None:
                try:
                    if notifications:
                        self.ros_notificator.update_task("Shutting down Gazebo web client",
                                                         update_progress=True, block_ui=False)
                    self.gzweb.stop()
                except Exception, e:
                    logger.warning("gzweb could not be stopped successfully")
                    logger.exception(e)

            if self.gzserver is not None:
                try:
                    if notifications:
                        self.ros_notificator.update_task("Shutting down Gazebo robotic simulator",
                                                         update_progress=True, block_ui=False)
                    self.gzserver.stop()
                except Exception, e:
                    logger.warning("gzserver could not be stopped successfully")
                    logger.exception(e)

            # Stop any external robot controllers
            if self.sim_config.ext_robot_controller:
                robot_controller_filepath = SimUtil.find_file_in_paths(
                    self.sim_config.ext_robot_controller, self.sim_config.model_paths)
                if robot_controller_filepath:
                    if notifications:
                        self.ros_notificator.update_task("Stopping external robot controllers",
                                                         update_progress=True, block_ui=False)
                    subprocess.check_call([robot_controller_filepath, 'stop'])

            # Stop any ROS nodes launched via roslaunch
            if self.sim_config.ros_launch_abs_path is not None and self.ros_launcher is not None:
                if notifications:
                    self.ros_notificator.update_task("Shutting down launched ROS nodes",
                                                     update_progress=True, block_ui=False)
                self.ros_launcher.shutdown()

            # try to notify for task completion, notificator should be valid until
            # the finally block below
            if notifications:
                self.ros_notificator.finish_task()

        finally:
            # always shut down the notificator ROS topics when done, no status for this
            # as there is no mechanism to deliver further updates
            try:
                if self.ros_notificator:
                    self.ros_notificator.shutdown()
            except Exception, e:
                logger.error("The ROS notificator could not be shut down")
                logger.exception(e)

            SimUtil.delete_simulation_dir()

        # Cleanup ROS core nodes, services, and topics (the command should be almost
        # instant and exit, but wrap it in a timeout since it's semi-officially supported)
        logger.info("Cleaning up ROS nodes and services")

        for param in rospy.get_param_names():
            if param not in self._initial_ros_params:
                rospy.delete_param(param)

        for node in rosnode.get_node_names():
            if node not in self._initial_ros_nodes:
                os.system('rosnode kill ' + str(node))

        try:
            res = subprocess.check_output(["rosnode", "list"])

            if res.find("/gazebo") > -1 and res.find("/Watchdog") > -1:
                os.system('rosnode kill /gazebo /Watchdog')

            elif res.find("/gazebo") > -1:
                os.system('rosnode kill /gazebo >/dev/null 2>&1')

            elif res.find("/Watchdog") > -1:
                os.system('rosnode kill /Watchdog >/dev/null 2>&1')
        except Exception, e:
            logger.exception(e)

        os.system("echo 'y' | timeout -s SIGKILL 10s rosnode cleanup >/dev/null 2>&1")

    def _shutdown(self, notifications):  # pragma: no cover
        """
        Shutdown the CLE and any hooks before shutting down Gazebo

        :param notifications: A flag indicating whether notifications should be attempted to send
        """

        raise NotImplementedError("This method must be overridden in inherited classes")
