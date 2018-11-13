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
This module contains the abstract base class of a simulation assembly using the CLE and Gazebo
"""

import logging
import netifaces
import os
import random
import subprocess
import sys
import tempfile
import zipfile
import json
logger = logging.getLogger(__name__)

from RestrictedPython import compile_restricted
from hbp_nrp_cle.cle import config
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import \
    get_all_neurons_as_dict, generate_tf, import_referenced_python_tfs, correct_indentation
from hbp_nrp_cleserver.server.SimulationAssembly import SimulationAssembly
from hbp_nrp_cleserver.server.ROSLaunch import ROSLaunch
from hbp_nrp_cleserver.server.LocalGazebo import LocalGazeboBridgeInstance, \
    LocalGazeboServerInstance
from hbp_nrp_cleserver.server.LuganoVizClusterGazebo import LuganoVizClusterGazebo, XvfbXvnError
from hbp_nrp_cleserver.server.GazeboSimulationRecorder import GazeboSimulationRecorder
from hbp_nrp_cle.robotsim.RobotManager import Robot, RobotManager
from hbp_nrp_backend import NRPServicesGeneralException

# These imports start NEST.
from hbp_nrp_cleserver.server.ROSCLEServer import ROSCLEServer
from hbp_nrp_cle.cle.ClosedLoopEngine import DeterministicClosedLoopEngine, ClosedLoopEngine
import hbp_nrp_cle.tf_framework as nrp
import hbp_nrp_cle.brainsim.config as brainconfig


models_path = os.environ.get('NRP_MODELS_DIRECTORY')


class GazeboSimulationAssembly(SimulationAssembly):
    """
    The abstract base class for a simulation assembly that uses Gazebo for world simulation
    """

    def __init__(self, sim_id, exc, bibi, **par):
        """
        Creates a new simulation assembly to simulate an experiment using the CLE and Gazebo
        :param sim_id: The simulation id
        :param exc: The experiment configuration
        :param bibi: The BIBI configuration
        :param gzserver_host: The gazebo host
        :param reservation: The reservation number
        :param timeout: The timeout for the simulation
        :param experiment_id: The experiment_id of the simulation
        """
        super(GazeboSimulationAssembly, self).__init__(sim_id, exc, bibi, par)

        if models_path is None:
            raise Exception("Server Error. NRP_MODELS_DIRECTORY not defined.")

        self.robotManager = RobotManager()

        # determine the Gazebo simulator target, due to dependencies this must
        # happen here
        gzserver_host = par.get('gzserver_host', 'local')
        timeout = par.get('timeout', None)
        reservation = par.get('reservation', None)
        experiment_id = par.get('experiment_id', None)

        if gzserver_host == 'local':
            self.gzserver = LocalGazeboServerInstance()
        elif gzserver_host == 'lugano':
            self.gzserver = LuganoVizClusterGazebo(
                timeout.tzinfo if timeout is not None else None, reservation
            )
        else:
            raise Exception(
                "The gzserver location '{0}' is not supported.", gzserver_host)

        self._timeout = timeout
        self.__gzserver_host = gzserver_host
        self.experiment_id = experiment_id
        self.gzweb = None
        self.ros_launcher = None
        self.gazebo_recorder = None

    def _start_gazebo(self, rng_seed, playback_path, extra_models, world_file):
        """
        Configures and starts the Gazebo simulator and backend services

        :param rng_seed: RNG seed to spawn Gazebo with
        :param playback_path: A path to playback information
        :param extra_models: An additional models path or None
        :param world_file: The world file that should be loaded by Gazebo
        """

        # Gazebo configuration and launch
        self._notify("Starting Gazebo robotic simulator")
        ifaddress = netifaces.ifaddresses(
            config.config.get('network', 'main-interface'))
        local_ip = ifaddress[netifaces.AF_INET][0]['addr']
        ros_master_uri = os.environ.get(
            "ROS_MASTER_URI").replace('localhost', local_ip)

        self.gzserver.gazebo_died_callback = self._handle_gazebo_shutdown

        # Physics engine selection from ExDConfig; pass as -e <physics_engine>
        # to gzserver
        physics_engine = self.exc.physicsEngine
        logger.info("Looking for physicsEngine tag value in ExDConfig")
        if physics_engine is not None:
            logger.info("Physics engine specified in ExDConfig: " +
                        str(repr(physics_engine)))
            # No need to check that the physics engine is valid, pyxb already
            # does that
        else:
            logger.info(
                "No physics engine specified explicitly. Using default setting 'ode'")
            physics_engine = "ode"

        # experiment specific gzserver command line arguments
        gzserver_args = '--seed {rng_seed} -e {engine} {world_file}'\
            .format(rng_seed=rng_seed,
                    engine=physics_engine,
                    world_file=world_file)

        # If playback is specified, load the first log/world file in the recording at Gazebo launch
        # TODO: when storage server is available this should be updated
        if playback_path:
            gzserver_args += ' --play {path}/gzserver/1.log'.format(
                path=playback_path)

        # optional roslaunch support prior to Gazebo launch
        if self.exc.rosLaunch is not None:

            # NRRPLT-5134, only local installs are currently supported
            if self.__gzserver_host != 'local':
                raise Exception(
                    'roslaunch is currently only supported on local installs.')

            self._notify(
                "Launching experiment ROS nodes and configuring parameters")
            self.ros_launcher = ROSLaunch(self.exc.rosLaunch.src)

        try:
            logger.info("gzserver arguments: " + gzserver_args)
            self.gzserver.start(ros_master_uri, extra_models, gzserver_args)
        except XvfbXvnError as exception:
            logger.error(exception)
            error = "Recoverable error occurred. Please try again. Reason: {0}".format(
                    exception)
            raise Exception(error)

        self._notify("Connecting to Gazebo robotic simulator")
        self.robotManager.init_scene_handler()

        self._notify("Connecting to Gazebo simulation recorder")
        self.gazebo_recorder = GazeboSimulationRecorder(self.sim_id)

        self._notify("Starting Gazebo web client")
        os.environ['GAZEBO_MASTER_URI'] = self.gzserver.gazebo_master_uri

        self.__set_env_for_gzbridge()

        # We do not know here in which state the previous user did let us
        # gzweb.
        self.gzweb = LocalGazeboBridgeInstance()
        self.gzweb.restart()

    # pylint: disable=missing-docstring
    # pylint: disable=broad-except
    def __set_env_for_gzbridge(self):
        def get_gzbridge_setting(name, default):
            """
                Obtain parameter from pyxb bindings of experiment schema.
                If something goes wrong return default value. The default
                also defines the type that the parameter should have.
            :param name: Name of the parameter
            :param default_: default value
            :return: a string
            """
            try:
                s = self.exc.gzbridgesettings
                val = getattr(s, name)
                val = type(default)(val)
            # pylint: disable=broad-except
            except Exception:
                val = default
            return repr(val)
        os.environ['GZBRIDGE_POSE_FILTER_DELTA_TRANSLATION'] \
            = get_gzbridge_setting('pose_update_delta_translation', 1.e-5)
        os.environ['GZBRIDGE_POSE_FILTER_DELTA_ROTATION'] = get_gzbridge_setting(
            'pose_update_delta_rotation', 1.e-4)
        os.environ['GZBRIDGE_UPDATE_EARLY_THRESHOLD'] = get_gzbridge_setting(
            'pose_update_early_threshold', 0.02)

    def _handle_gazebo_shutdown(self):  # pragma: no cover
        """
        Handles the case that gazebo died unexpectedly
        """
        logger.exception("Gazebo died unexpectedly")
        # Avoid further notice
        self.gzserver.gazebo_died_callback = None
        # in case the simulation is still being started, we abort the
        # initialization
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
        if self.exc.rosLaunch is not None:
            number_of_subtasks += 1
        if self.bibi.extRobotController is not None:
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
                    logger.warning(
                        "Gazebo recorder could not be shutdown successfully")
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
                    logger.warning(
                        "gzserver could not be stopped successfully")
                    logger.exception(e)

            # Stop any external robot controllers
            if self.bibi.extRobotController:
                from hbp_nrp_backend.storage_client_api.StorageClient \
                    import get_model_basepath, find_file_in_paths
                robot_controller_filepath = find_file_in_paths(self.bibi.extRobotController,
                                                               get_model_basepath())
                if robot_controller_filepath:
                    if notifications:
                        self.ros_notificator.update_task("Stopping external robot controllers",
                                                         update_progress=True, block_ui=False)
                    subprocess.check_call([robot_controller_filepath, 'stop'])

            # Stop any ROS nodes launched via roslaunch
            if self.exc.rosLaunch is not None and self.ros_launcher is not None:
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

        # Cleanup ROS core nodes, services, and topics (the command should be almost
        # instant and exit, but wrap it in a timeout since it's semi-officially
        # supported)
        logger.info("Cleaning up ROS nodes and services")

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


class CLEGazeboSimulationAssembly(GazeboSimulationAssembly):
    """
    This class assembles the simulation using the CLE
    """

    def __init__(self, sim_id, exc, bibi, **par):
        """
        Creates a new simulation assembly to simulate an experiment using the CLE and Gazebo
        :param sim_id: The simulation id
        :param exc: The experiment configuration
        :param bibi: The BIBI configuration
        """
        super(CLEGazeboSimulationAssembly, self).__init__(sim_id, exc, bibi, **par)
        self.cle_server = None
        self.tempAssetsDir = 'assets'

        # FIXME: fix dependency loop from the imports
        from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient
        self._storageClient = StorageClient()
        self._simDir = self._storageClient.get_simulation_directory()

    @property
    def simdir(self):
        """
        Gets the simulation directory
        """
        return self._simDir

    @property
    def storage_client(self):
        """
        Gets the storage client handler
        """
        return self._storageClient

    def _initialize(self, environment, except_hook):
        """
        Internally initialize the simulation
        :param environment: The environment that should be simulated
        :param except_hook: A method that should be called when there is a critical error
        """
        # pylint: disable=too-many-locals

        # create the CLE server and lifecycle first to report any failures properly
        logger.info("Creating CLE Server")
        self.cle_server = ROSCLEServer(self.sim_id, self._timeout, self.gzserver,
                                       self.ros_notificator)

        # RNG seed for components, use config value if specified or generate a new one
        rng_seed = self.exc.rngSeed
        if rng_seed is None:
            logger.warn(
                'No RNG seed specified, generating a random value.')
            rng_seed = random.randint(1, sys.maxint)
        logger.info('RNG seed = %i', rng_seed)

        # start Gazebo simulator and bridge
        extraModelDirs = os.path.join(self._simDir, self.tempAssetsDir) + ':' + self._simDir
        playbackPath = None
        self._start_gazebo(rng_seed, playbackPath, extraModelDirs, environment)

        # load environment and robot models
        models, lights = self._load_environment(environment)

        # find robot
        robot_poses = {}
        self.robotManager.remove_all_robots()
        self.__read_robot_list()

        for rid, robot in self.robotManager.get_robot_dict().iteritems():
            self._load_robot(robot)
            robot_poses[rid] = robot.pose

        # load robot adapters
        robotcomm, robotcontrol = self._create_robot_adapters()

        # load the brain
        braincontrol, braincomm, brainfile, brainconf = self._load_brain(rng_seed)

        # initialize the cle server and services
        logger.info("Preparing CLE Server")
        self.cle_server.cle = self.__load_cle(robotcontrol, robotcomm, braincontrol, braincomm,
                                              brainfile, brainconf, robot_poses, models, lights)
        self.cle_server.prepare_simulation(except_hook)
        self.cle_server.setup_handlers(self)

        # load transfer functions
        self.__load_tfs()

        # Wait for the backend rendering environment to load (for any sensors/cameras)
        self._notify("Waiting for Gazebo simulated sensors to be ready")
        self.robotManager.scene_handler().wait_for_backend_rendering()

    # TODO: remove this function when exc and bibi abstraction (SimConf) is implemented
    def __read_robot_list(self):
        """
        Reads robot list from bibi and poses from exc and populates robot manager

        :return: -
        """
        # pylint: disable=too-many-branches
        if (not self.bibi.bodyModel):
            return

        for modelTag in self.bibi.bodyModel:
            if (modelTag.robotId is None):
                modelTag.robotId = 'robot'
            elif (not modelTag.robotId
                  or modelTag.robotId in self.robotManager.get_robot_dict()):
                raise Exception("Multiple bodyModels has been defined with same or no names."
                                "Please check bibi config file.")

            pose = None
            path = None
            isCustom = False
            for rpose in self.exc.environmentModel.robotPose:
                if not (rpose.robotId) or rpose.robotId == modelTag.robotId:
                    pose = RobotManager.convertXSDPosetoPyPose(rpose)

            if hasattr(modelTag, 'customModelPath') and modelTag.customModelPath is not None:
                # this tag is never there, remove this if block?
                modelTag.assetPath = modelTag.customModelPath
                isCustom = modelTag.customAsset = True
            elif hasattr(modelTag, 'customAsset') and modelTag.customAsset is not None:
                isCustom = modelTag.customAsset
            else:
                modelTag.customAsset = False

            if hasattr(modelTag, 'assetPath') and modelTag.assetPath is None:
                modelTag.assetPath = "."

            from hbp_nrp_backend.storage_client_api.StorageClient import \
                find_file_in_paths, get_model_basepath

            if isCustom:
                path = self._extract_robot_zip(modelTag)
            else:
                path = find_file_in_paths(os.path.join(modelTag.robotId,
                                                       os.path.basename(modelTag.value())),
                                          get_model_basepath())
                # Perhaps it's a previously coned experiment? Try with modelTag.value() BUT
                # only look into the simulation_directory, as DELETE robot REST call, if called,
                # would delete this file
                # TODO: backward compatibility code. Remove when we decide not to support anymore
                if not path:
                    path = find_file_in_paths(modelTag.value(), [self._simDir])

            # still couldn't find the SDF, abort!
            if not path:
                raise Exception("Could not find robot file: {0}".format(modelTag.value()))
            self.robotManager.add_robot(
                Robot(modelTag.robotId, path, modelTag.robotId, pose, isCustom)
            )

    def _extract_robot_zip(self, robot_file):
        """
        Get the zip from the storage and extract it in the current simulation directory

        :param robot_file: DOM object of <bodyModel> in bibi.
            Contains SDF path for template robots as value.
            Or zip path for custom robot as 'assetPath' and SDF location with the zip as value.
        :return Absolute path to the robot SDF
        """
        # pylint: disable=too-many-locals
        robots_list = self._storageClient.get_custom_models(self.token, self.ctx_id, 'robots')

        # we use the paths of the uploaded zips to make sure the selected zip is there
        paths_list = [robot['path'] for robot in robots_list]

        # FIXME: change this implementation. it would fail if there're awesome_husky.zip and \
        # a husky.zip, and assetPath is set to husky.zip
        # check if the zip is in the user storage
        zipped_model_path = [path for path in paths_list if robot_file.assetPath in path]
        if zipped_model_path:
            model_data = {'uuid': zipped_model_path[0]}
            json_model_data = json.dumps(model_data)
            storage_robot_zip_data = self._storageClient.get_custom_model(
                self.token,
                self.ctx_id, json_model_data)

            rob_zip_path = os.path.join(self._simDir, robot_file.assetPath)
            with open(rob_zip_path, 'w') as robot_zip:
                robot_zip.write(storage_robot_zip_data)

            with zipfile.ZipFile(rob_zip_path) as robot_zip_to_extract:
                robot_zip_to_extract.extractall(path=os.path.join(self._simDir, self.tempAssetsDir))

        # if the zip is not there, prompt the user to check his uploaded models
        else:
            raise NRPServicesGeneralException(
                "Could not find zip file {0} in the list of uploaded models. "
                .format(robot_file.value()) +
                "Please make sure that it has been uploaded correctly.",
                "Zipped model retrieval failed")

        # TODO: move this statement from here
        return os.path.join(self._simDir, robot_file.value())

    # pylint: disable-msg=too-many-branches
    def _load_environment(self, world_file):
        """
        Loads the environment and robot in Gazebo

        :param world_file Backwards compatibility for world file specified through webpage
        """
        # pylint: disable=too-many-locals

        # load the world file if provided first
        self._notify("Loading experiment environment")
        w_models, w_lights = self.robotManager.scene_handler() \
            .parse_gazebo_world_file(world_file)

        return w_models, w_lights

    # pylint: disable-msg=too-many-branches
    def _load_robot(self, robot):
        """
        Loads the environment and robot in Gazebo

        :param robot Robot object to load
        """
        # pylint: disable=too-many-locals
        # Create interfaces to Gazebo
        self._notify("Loading robot")
        logger.info("RobotAbs: " + str(robot.SDFFileAbsPath))

        # check retina script file
        retina_config_path = None
        for conf in self.bibi.configuration:
            if conf.type == 'retina':
                self._notify("Configuring Retina Camera Plugin")
                retina_config_path = conf.src

        # spawn robot model
        self.robotManager.load_robot_in_scene(robot.id, retina_config_path)

        # load external robot controller
        if self.bibi.extRobotController is not None:
            from hbp_nrp_backend.storage_client_api.StorageClient \
                import get_model_basepath, find_file_in_paths
            robot_controller_filepath = find_file_in_paths(self.bibi.extRobotController,
                                                           get_model_basepath())
            if not os.path.isfile(robot_controller_filepath) and self._simDir is not None:
                robot_controller_filepath = os.path.join(self._simDir,
                                                         self.bibi.extRobotController)
            if os.path.isfile(robot_controller_filepath):
                self._notify("Loading external robot controllers")  # +1
                res = subprocess.call([robot_controller_filepath, 'start'])
                if res > 0:
                    logger.error(
                        "The external robot controller could not be loaded")
                    self.shutdown()
                    return

    def _create_robot_adapters(self):  # pragma: no cover
        """
        Creates the adapter components for the robot side

        :return: A tuple of the communication and control adapter for the robot side
        """
        raise NotImplementedError("This method must be overridden in an implementation")

    def _load_brain(self, rng_seed):
        """
        Loads the neural simulator, interfaces, and configuration

        :param rng_seed RNG seed to spawn Nest with
        """

        # Create interfaces to brain
        self._notify("Loading neural simulator")
        brainconfig.rng_seed = rng_seed
        braincomm, braincontrol = self._create_brain_adapters()

        self._notify("Loading brain and population configuration")
        # load brain

        # find robot
        if not self.bibi.brainModel:
            return braincontrol, braincomm, None, None

        brainfilepath = self.bibi.brainModel.file
        if self.bibi.brainModel.customModelPath:
            self._extract_brain_zip()
        if self.__is_collab_hack():
            if self.exc.dir is not None:
                brainfilepath = os.path.join(self.exc.dir, brainfilepath)
        elif 'storage://' in brainfilepath:
            brainfilepath = os.path.join(self._simDir, os.path.basename(brainfilepath))
            with open(brainfilepath, "w") as f:
                f.write(self._storageClient.get_file(
                    self.token,
                    self._storageClient.get_folder_uuid_by_name(self.token, self.ctx_id, 'brains'),
                    os.path.basename(brainfilepath),
                    byname=True))
        else:
            from hbp_nrp_backend.storage_client_api.StorageClient \
                import get_model_basepath, find_file_in_paths
            brainfilepath = find_file_in_paths(brainfilepath, get_model_basepath())

            #if not brainfilepath:
            #    raise Exception("Could not find brain file: ".format(brainfilepath))

        neurons_config = get_all_neurons_as_dict(
            self.bibi.brainModel.populations)

        return braincontrol, braincomm, brainfilepath, neurons_config

    def _extract_brain_zip(self):
        """
        Checks for validity, and extracts a zipped brain. First we
        make sure that the zip referenced in the bibi exists in the
        list of user brains, then we unzip it on the fly in the temporary
        simulation directory. After the extraction we also make sure to copy
        the .py from the experiment folder cause the user may have modified it
        """
        # pylint: disable=too-many-locals
        brains_list = self._storageClient.get_custom_models(self.token, self.ctx_id, 'brains')
        # we use the paths of the uploaded zips to make sure the selected
        # zip is there
        paths_list = [brain['path'] for brain in brains_list]

        # check if the zip is in the user storage
        zipped_model_path = [
            path for path in paths_list if self.bibi.brainModel.customModelPath in path]
        if len(zipped_model_path):
            model_data = {'uuid': zipped_model_path[0]}
            json_model_data = json.dumps(model_data)
            storage_brain_zip_data = self._storageClient.get_custom_model(self.token,
                                                                          self.ctx_id,
                                                                          json_model_data)
            brain_name = os.path.basename(self.bibi.brainModel.file)
            brn_zip_path = os.path.join(self._simDir, self.bibi.brainModel.customModelPath)
            with open(brn_zip_path, 'w') as brain_zip:
                brain_zip.write(storage_brain_zip_data)
            with zipfile.ZipFile(brn_zip_path) as brain_zip_to_extract:
                for brain_file in brain_zip_to_extract.namelist():
                    _, file_name = os.path.split(brain_file)
                    if file_name:
                        with open(os.path.join(self._simDir, file_name), 'w') as file_to_write:
                            file_to_write.write(brain_zip_to_extract.read(brain_file))

            # copy back the .py from the experiment folder, cause we don't want the one
            # in the zip, cause the user might have made manual changes
            self._storageClient.clone_file(brain_name, self.token, self.experiment_id)
        # if the zip is not there, prompt the user to check his uploaded
        # models
        else:
            raise NRPServicesGeneralException(
                "Could not find selected zip {zip} in the list of uploaded models. ".format(
                    zip=os.path.dirname(self.bibi.brainModel.customModelPath)
                ) + "Please make sure that it has been uploaded correctly",
                "Zipped model retrieval failed")

    def _create_brain_adapters(self):  # pragma: no cover
        """
        Creates the adapter components for the neural simulator

        :return: A tuple of the communication and control adapter for the neural simulator
        """
        raise NotImplementedError("This method must be overridden in an implementation")

    # pylint: disable=too-many-arguments
    def __load_cle(self, roscontrol, roscomm, braincontrol, braincomm,
                   brain_file_path, neurons_config,
                   robot_poses, models, lights):
        """
        Load the ClosedLoopEngine and initializes all interfaces

        :param roscontrol Robot Control Adapter to use
        :param roscomm Robot Communication Adapter to use
        :param braincontrol Brain Control Adapter to use
        :param braincomm Brain Communication Adapter to use
        :param brain_file_path Accessible path to brain file
        :param neurons_config Neuron configuration specified in the BIBI
        :param robot_post Initial robot pose
        :param models Initial models loaded into the environment
        :param lights Initial lights loaded into the environment
        """

        # Needed in order to cleanup global static variables
        self._notify("Connecting brain simulator to robot")
        nrp.start_new_tf_manager()

        # Create transfer functions manager
        tfmanager = nrp.config.active_node

        # set adapters
        tfmanager.robot_adapter = roscomm
        tfmanager.brain_adapter = braincomm

        # integration timestep between simulators, convert from ms to s
        # (default to CLE value)
        timestep = ClosedLoopEngine.DEFAULT_TIMESTEP
        if self.bibi.timestep is not None:
            timestep = float(self.bibi.timestep) / 1000.0

        roscontrol.set_robots(self.robotManager.get_robot_dict())

        # initialize CLE
        self._notify("Initializing CLE")

        cle = DeterministicClosedLoopEngine(roscontrol, roscomm,
                                            braincontrol, braincomm,
                                            tfmanager, timestep
                                            )

        if (brain_file_path):
            cle.initialize(brain_file_path, **neurons_config)
        else:
            cle.initialize()

        # Set initial pose
        cle.initial_robot_poses = robot_poses
        # Set initial models and lights
        cle.initial_models = models
        cle.initial_lights = lights

        return cle

    def __load_tfs(self):
        """
        Loads and connects all transfer functions
        """
        self._notify("Loading transfer functions")

        # Create transfer functions
        import_referenced_python_tfs(self.bibi, self.exc.dir)

        for i, tf in enumerate(self.bibi.transferFunction):
            self._notify("Generating transfer function: %i" % (i + 1))
            tf_code = generate_tf(tf)
            self._notify("Loading transfer function: %s" % tf.name)
            tf_code = correct_indentation(tf_code, 0)
            tf_code = tf_code.strip() + "\n"
            logger.debug("TF: " + tf.name + "\n" + tf_code + '\n')

            try:
                new_code = compile_restricted(tf_code, '<string>', 'exec')
            # pylint: disable=broad-except
            except Exception as e:
                message = "Error while compiling the updated transfer function named "\
                          + tf.name +\
                          " in restricted mode.\n"\
                          + str(e)
                logger.error(message)
                nrp.set_flawed_transfer_function(tf_code, tf.name, e)
                continue

            try:
                nrp.set_transfer_function(tf_code, new_code, tf.name)
            except nrp.TFLoadingException as loading_e:
                logger.error(loading_e)
                nrp.set_flawed_transfer_function(tf_code, tf.name, loading_e)

    def _handle_gazebo_shutdown(self):
        """
        Handles the case that Gazebo was shut down

        :param close_error: Any exception happened while closing Gazebo
        """
        super(CLEGazeboSimulationAssembly, self)._handle_gazebo_shutdown()
        if self.cle_server is not None and self.cle_server.lifecycle is not None:
            # Set the simulation to halted
            self.cle_server.lifecycle.failed()
            # If not already stopped, free simulation resources
            self.cle_server.lifecycle.stopped()

    def run(self):
        """
        Runs the simulation
        """
        self.cle_server.run()

    def _shutdown(self, notifications):
        """
        Shutdown the CLE and any hooks before shutting down Gazebo

        :param notifications: A flag indicating whether notifications should be attempted to send
        """
        try:
            if notifications:
                self.ros_notificator.update_task("Shutting down Closed Loop Engine",
                                                 update_progress=True, block_ui=False)

            self.cle_server.shutdown()
        # pylint: disable=broad-except
        except Exception, e:
            logger.error("The cle server could not be shut down")
            logger.exception(e)

        finally:
            self._storageClient.remove_temp_directory()

    def __is_collab_hack(self):
        """
        This horrible hack is supposed to be dropped when we remove support for SDF cloning
        when we have introduced robot and env libraries.
        :return: true if we detect we are in collab models
        """
        return self.exc.dir.startswith(tempfile.gettempdir())
