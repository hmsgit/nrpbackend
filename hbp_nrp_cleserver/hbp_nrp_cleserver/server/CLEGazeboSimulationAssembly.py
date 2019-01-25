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
import os
import random
import subprocess
import sys
import tempfile
import zipfile
import json
logger = logging.getLogger(__name__)

from RestrictedPython import compile_restricted
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import \
    get_all_neurons_as_dict, generate_tf, import_referenced_python_tfs, correct_indentation
from hbp_nrp_cle.robotsim.RobotManager import Robot, RobotManager
from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient, \
    get_model_basepath, find_file_in_paths
from hbp_nrp_cleserver.server.GazeboSimulationAssembly import GazeboSimulationAssembly
from hbp_nrp_commons.ZipUtil import ZipUtil

# These imports start NEST.
from hbp_nrp_cleserver.server.ROSCLEServer import ROSCLEServer
from hbp_nrp_cle.cle.ClosedLoopEngine import DeterministicClosedLoopEngine, ClosedLoopEngine
import hbp_nrp_cle.tf_framework as nrp
import hbp_nrp_cle.brainsim.config as brainconfig


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
        # initialize the cle server and services
        logger.info("Creating CLE Server")
        self.cle_server = ROSCLEServer(self.sim_id, self._timeout, self.gzserver,
                                       self.ros_notificator)
        self.cle_server.setup_handlers(self)

        # RNG seed for components, use config value if specified or generate a new one
        rng_seed = self.exc.rngSeed
        if rng_seed is None:
            logger.warn('No RNG seed specified, generating a random value.')
            rng_seed = random.randint(1, sys.maxint)
        logger.info('RNG seed = %i', rng_seed)

        # start Gazebo simulator and bridge
        extra_model_dirs = os.path.join(self._simDir, self.tempAssetsDir) + ':' + self._simDir
        playback_path = None
        self._start_gazebo(rng_seed, playback_path, extra_model_dirs, environment)

        # load user textures in Gazebo
        self._load_textures()

        # load environment and robot models
        models, lights = self._load_environment(environment)

        # find robot
        robot_poses = {}
        self.robotManager.remove_all_robots()

        self._load_robot()
        for rid, robot in self.robotManager.get_robot_dict().iteritems():
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

        # load transfer functions
        self.__load_tfs()

        # Wait for the backend rendering environment to load (for any sensors/cameras)
        self._notify("Waiting for Gazebo simulated sensors to be ready")
        self.robotManager.scene_handler().wait_for_backend_rendering()
        # Spawns a new thread for the csv logger
        # pylint: disable=protected-access
        self.cle_server._csv_logger.initialize()

    # TODO: remove this function when exc and bibi abstraction (SimConf) is implemented
    # TODO: remove code duplication with _RobotCallHandler once TODO mentioned inside is resolved
    def _add_bibi_robots(self):
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
            elif (not modelTag.robotId or modelTag.robotId in self.robotManager.get_robot_dict()):
                raise Exception("Multiple bodyModels has been defined with same or no names. "
                                "Please check bibi config file.")

            pose = None
            path = None
            isCustom = False
            for rpose in self.exc.environmentModel.robotPose:
                if not (rpose.robotId) or rpose.robotId == modelTag.robotId:
                    pose = RobotManager.convertXSDPosetoPyPose(rpose)

            if hasattr(modelTag, 'customAsset') and modelTag.customAsset is not None:
                isCustom = modelTag.customAsset
            else:
                modelTag.customAsset = False

            if isCustom:
                if not hasattr(modelTag, "assetPath"):
                    raise Exception("No zipped model path is provided in bibi.bodyModel.assetPath")
                # pylint: disable=protected-access
                downloadedZipPath = self.cle_server._robotHandler.download_custom_robot(
                    modelTag.assetPath,
                    self._simDir,
                    modelTag.assetPath)
                if downloadedZipPath is not None:
                    ZipUtil.extractall(
                        zip_abs_path=downloadedZipPath,
                        extract_to=os.path.join(self._simDir, self.tempAssetsDir),
                        overwrite=True)

                    path = os.path.join(self._simDir, modelTag.value())

            else:
                path = find_file_in_paths(os.path.join(
                    modelTag.robotId, os.path.basename(modelTag.value())), get_model_basepath())

                # Perhaps it's a previously coned experiment? Try with modelTag.value() BUT
                # only look into the simulation_directory, as DELETE robot REST call, if called,
                # would delete this file
                # TODO: backward compatibility code. Remove when we decide not to support anymore
                if not path:
                    path = find_file_in_paths(modelTag.value(), [self._simDir])

            # still couldn't find the SDF, abort!
            if not path:
                raise Exception("Could not find robot file: {0}".format(modelTag.value()))

            # Find robot specific roslaunch file in the directory where the SDF resides
            # Take the first one (by name) if multiple available
            rosLaunchRelPath = next((f for f in os.listdir(os.path.dirname(path))
                                     if f.endswith('.launch')), None)
            rosLaunchAbsPath = (None if rosLaunchRelPath is None
                                else os.path.join(os.path.dirname(path), rosLaunchRelPath))

            self.robotManager.add_robot(
                Robot(modelTag.robotId, path, modelTag.robotId, pose, isCustom, rosLaunchAbsPath)
            )

    def _load_environment(self, world_file):
        """
        Loads the environment and robot in Gazebo
        :param world_file Backwards compatibility for world file specified through webpage
        """
        # load the world file if provided first
        self._notify("Loading experiment environment")
        return self.robotManager.scene_handler().parse_gazebo_world_file(world_file)

    def _load_textures(self):
        """
        Loads custom textures in Gazebo
        """
        self._notify("Loading textures")

        try:
            textures = self.storage_client.get_textures(self.experiment_id, self.token)
        except:  # pylint: disable=bare-except
            logger.info("Non-existent textures or folder!")
            return  # ignore missing textures or texture folder

        try:
            if textures:
                self.robotManager.scene_handler().load_textures(textures)
        except:  # pylint: disable=bare-except
            logger.info("Timeout while trying to load textures.")

    # pylint: disable-msg=too-many-branches
    def _load_robot(self):
        """
        Loads robots defined in the bibi and initializes any external controller
        """
        # Set retina config for the robotManager
        for conf in self.bibi.configuration:
            if conf.type == 'retina':
                self._notify("Configuring Retina Camera Plugin")
                self.robotManager.retina_config = conf.src

        self._notify("Loading robots")
        self._add_bibi_robots()

        # load external robot controller
        if self.bibi.extRobotController is not None:
            robot_controller_filepath = find_file_in_paths(self.bibi.extRobotController,
                                                           get_model_basepath())
            if not os.path.isfile(robot_controller_filepath) and self._simDir is not None:
                robot_controller_filepath = os.path.join(self._simDir,
                                                         self.bibi.extRobotController)
            if os.path.isfile(robot_controller_filepath):
                self._notify("Loading external robot controllers")  # +1
                res = subprocess.call([robot_controller_filepath, 'start'])
                if res > 0:
                    logger.error("The external robot controller could not be loaded")
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
                    by_name=True))
        else:
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
                                            tfmanager, timestep)

        if brain_file_path:
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
                self.ros_notificator.update_task(
                    "Shutting down Closed Loop Engine", update_progress=True, block_ui=False)

            self.robotManager.shutdown()
            self.cle_server.shutdown()
        # pylint: disable=broad-except
        except Exception, e:
            logger.error("The cle server could not be shut down")
            logger.exception(e)
        finally:
            self._storageClient.remove_temp_sim_directory()
        # kill the csv logger thread
        # pylint: disable=protected-access
        self.cle_server._csv_logger.shutdown()

    def __is_collab_hack(self):
        """
        This horrible hack is supposed to be dropped when we remove support for SDF cloning
        when we have introduced robot and env libraries.
        :return: true if we detect we are in collab models
        """
        return self.exc.dir.startswith(tempfile.gettempdir())
