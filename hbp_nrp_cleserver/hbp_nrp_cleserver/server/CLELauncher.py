# -*- coding: utf-8 -*-
"""
CLELauncher starts a new CLE. It substitutes the old cle_template.pyt
"""

__author__ = 'Lorenzo Vannuci, Georg Hinkel, Bernd Eckstein'

import os
import netifaces
import subprocess
import logging
import importlib

from RestrictedPython import compile_restricted

from geometry_msgs.msg import Pose

from hbp_nrp_cleserver import config

from hbp_nrp_cleserver.bibi_config.notificator import Notificator, NotificatorHandler
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import compute_dependencies
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import \
    get_all_neurons_as_dict, generate_tf, import_referenced_python_tfs, correct_indentation

from hbp_nrp_commons.generated import bibi_api_gen
from hbp_nrp_commons.generated import exp_conf_api_gen

from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter
from hbp_nrp_cle.robotsim.LocalGazebo import LocalGazeboBridgeInstance, LocalGazeboServerInstance
from hbp_nrp_cle.robotsim.LuganoVizClusterGazebo import LuganoVizClusterGazebo
from hbp_nrp_cle.robotsim.GazeboLoadingHelper import \
    load_gazebo_model_file, empty_gazebo_world, load_gazebo_world_file

# These imports start NEST.
from hbp_nrp_cleserver.server.ROSCLEServer import ROSCLEServer
from hbp_nrp_cle.cle.ClosedLoopEngine import ClosedLoopEngine
from hbp_nrp_cle.brainsim import instantiate_communication_adapter
from hbp_nrp_cle.brainsim import instantiate_control_adapter
import hbp_nrp_cle.tf_framework as nrp
# End of NEST-starting imports

logger = logging.getLogger(__name__)

# pylint: disable = too-many-locals
# pylint: disable = too-many-statements
# pylint: disable = too-many-branches


class CLELauncher(object):
    """
    CLELauncher substitutes generated cle
    """

    def __init__(self, exdConf, bibiConf, tf_path, gzserver_host, sim_id):
        """
        Constructor of CLELauncher

        @param exdConf: Experiment Configuration
        @type exdConf: generated_experiment_api.ExD
        @param bibiConf: Bibi Configuration
        @type bibiConf: generated_bibi_api.BIBIConfiguration
        @param tf_path: path for transfer functions
        @param gzserver_host: 'local' or 'lugano'
        @type gzserver_host: str
        @param sim_id: simulation id
        @type sim_id: int
        @return: [cle_server, models_path, gzweb, gzserver]
        """

        assert isinstance(exdConf, exp_conf_api_gen.ExD_)
        assert isinstance(bibiConf, bibi_api_gen.BIBIConfiguration)

        self.exdConf = exdConf
        self.bibiConf = bibiConf

        self.tf_path = tf_path
        self.gzserver_host = gzserver_host
        self.sim_id = sim_id
        self.robot_initial_pose = exdConf.environmentModel.robotPose
        self.timeout = exdConf.timeout
        self.dependencies = compute_dependencies(bibiConf)

    def cle_function_init(self, world_file):
        """
        init cle

        @param world_file:
        @return:
        """

        # Create ROS server

        logger.info("Creating ROSCLEServer")
        cle_server = ROSCLEServer(self.sim_id)
        logger.info("Setting up backend Notificator")
        Notificator.register_notification_function(
            lambda subtask, update_progress:
            cle_server.notify_current_task(subtask, update_progress, True)
        )

        # We use the logger hbp_nrp_cle.user_notifications in the CLE to log
        # information that is useful to know for the user.
        # In here, we forward any info message sent to this logger to the notificator
        gazebo_logger = logging.getLogger('hbp_nrp_cle.user_notifications')
        gazebo_logger.setLevel(logging.INFO)
        gazebo_logger.handlers.append(NotificatorHandler())

        # Only for Frontend progress bar logic
        number_of_subtasks = 9
        if self.bibiConf.extRobotController is not None:
            number_of_subtasks = 10

        cle_server.notify_start_task("Initializing the Neurorobotic Closed Loop Engine",
                                     "Importing needed packages",
                                     number_of_subtasks=number_of_subtasks,
                                     block_ui=True)

        # Needed in order to cleanup global static variables
        nrp.start_new_tf_manager()

        # consts
        TIMESTEP = 0.02

        # set models path variable
        models_path = get_basepath()

        Notificator.notify("Resetting Gazebo robotic simulator", True)  # subtask 1

        ifaddress = netifaces.ifaddresses(config.config.get('network', 'main-interface'))
        local_ip = ifaddress[netifaces.AF_INET][0]['addr']
        ros_master_uri = os.environ.get("ROS_MASTER_URI").replace('localhost', local_ip)

        gzweb = LocalGazeboBridgeInstance()

        gzserver = None
        if self.gzserver_host == 'local':
            gzserver = LocalGazeboServerInstance()
            gzserver.start(ros_master_uri)
        elif self.gzserver_host == 'lugano':
            gzserver = LuganoVizClusterGazebo()
            gzserver.start(ros_master_uri)

        if gzserver is None:
            logger.error("No configuration found for gzserver_host: "
                         "'{0}'".format(self.gzserver_host))
            self.shutdown(cle_server, None, None, None)
            return [None, None, None, None]

        os.environ['GAZEBO_MASTER_URI'] = gzserver.gazebo_master_uri
        # We do not know here in which state the previous user did let us gzweb.
        gzweb.restart()

        empty_gazebo_world()

        Notificator.notify("Loading experiment environment", True)  # subtask 2

        load_gazebo_world_file(world_file)

        # Create interfaces to Gazebo
        Notificator.notify("Loading neuRobot", True)  # subtask 3

        if self.robot_initial_pose is not None:
            rpose = Pose()
            rpose.position.x = self.robot_initial_pose.x
            rpose.position.y = self.robot_initial_pose.y
            rpose.position.z = self.robot_initial_pose.z
            rpose.orientation.x = self.robot_initial_pose.ux
            rpose.orientation.y = self.robot_initial_pose.uy
            rpose.orientation.z = self.robot_initial_pose.uz
            rpose.orientation.w = self.robot_initial_pose.theta
        else:
            rpose = None

        # spawn robot model
        load_gazebo_model_file('robot', self.bibiConf.bodyModel, rpose)

        # control adapter
        roscontrol = RosControlAdapter()
        # communication adapter
        roscomm = RosCommunicationAdapter()

        if self.bibiConf.extRobotController is not None:  # load external robot controller
            robot_controller_filepath = os.path.join(models_path, self.bibiConf.extRobotController)
            if os.path.isfile(robot_controller_filepath):
                Notificator.notify("Loading external robot controllers", True)  # +1
                res = subprocess.call([robot_controller_filepath, 'start'])
                if res > 0:
                    logger.error("The external robot controller could not be loaded")
                    self.shutdown(cle_server, None, None, None)
                    return [None, None, None, None]

        # Create interfaces to brain
        Notificator.notify("Loading neural Simulator NEST", True)  # subtask 4
        # control adapter
        braincontrol = instantiate_control_adapter()
        # communication adapter
        braincomm = instantiate_communication_adapter()
        # Create transfer functions manager
        Notificator.notify("Connecting neural simulator to neurobot", True)  # subtask 5
        # tf manager
        tfmanager = nrp.config.active_node

        # set adapters
        tfmanager.robot_adapter = roscomm
        tfmanager.brain_adapter = braincomm

        # Import dependencies
        for dep in self.dependencies:
            importlib.import_module(dep[:dep.rfind('.')])

        # Create CLE
        cle = ClosedLoopEngine(roscontrol, roscomm, braincontrol, braincomm, tfmanager, TIMESTEP)

        Notificator.notify("Loading Brain", True)  # subtask 6
        # load brain
        brainfilepath = self.bibiConf.brainModel.file
        if models_path is not None:
            brainfilepath = os.path.join(models_path, brainfilepath)

        # initialize everything
        neurons_config = get_all_neurons_as_dict(self.bibiConf.brainModel.populations)
        cle.load_brain(brainfilepath, **neurons_config)

        Notificator.notify("Initializing CLE", True)  # subtask 7
        cle.initialize()

        # Set initial pose
        cle.initial_robot_pose = rpose

        # Now that we have everything ready, we could prepare the simulation
        cle_server.prepare_simulation(cle, self.timeout)

        Notificator.notify("Injecting Transfer Functions", True)  # subtask 8

        # Create transfer functions
        import_referenced_python_tfs(self.bibiConf, self.tf_path)

        for tf in self.bibiConf.transferFunction:
            tf_code = generate_tf(tf)
            tf_code = correct_indentation(tf_code, 0)
            tf_code = tf_code.strip() + "\n"
            logger.info("TF: " + tf.name + "\n" + tf_code + '\n')

            try:
                new_code = compile_restricted(tf_code, '<string>', 'exec')
                nrp.set_transfer_function(tf_code, new_code, tf.name)
            # pylint: disable=broad-except
            except Exception as e:
                logger.error("Error while loading new transfer function")
                logger.error(e)

        # Loading is completed.
        Notificator.notify("Finished", True)  # subtask 9
        cle_server.notify_finish_task()

        logger.info("CLELauncher Finished.")

        return [cle_server, models_path, gzweb, gzserver]

    def shutdown(self, cle_server, models_path, gzweb, gzserver):
        """
        Shutdown CLE

        @param cle_server: the cle_server
        @param models_path: the models matp
        @param gzweb: gzweb
        @param gzserver: gzserver
        @return:
        """

        # Once we do reach this point, the simulation is stopped and we could clean after ourselves.
        # Clean up gazebo after ourselves
        cle_server.notify_start_task("Stopping simulation",
                                     "Emptying 3D world",
                                     number_of_subtasks=2,
                                     block_ui=False)
        empty_gazebo_world()

        if gzweb is not None:
            gzweb.stop()
        if gzserver is not None:
            gzserver.stop()

        if self.bibiConf.extRobotController is not None:  # optionally stop all external robot
            # controllers
            robot_controller_filepath = os.path.join(models_path, self.bibiConf.extRobotController)
            if os.path.isfile(robot_controller_filepath):
                cle_server.notify_current_task("Stopping external robot controllers",
                                               update_progress=True, block_ui=False)
                subprocess.check_call([robot_controller_filepath, 'stop'])

        # Shutdown CLE
        Notificator.register_notification_function(
            lambda subtask, update_progress: cle_server.notify_current_task(subtask,
                                                                            update_progress, False)
        )
        cle_server.notify_current_task("Shutting down Closed Loop Engine",
                                       update_progress=True, block_ui=False)

        # we could close the notify task here but it will be closed in any case by shutdown()
        cle_server.shutdown()
        # shutdown is complete


def get_basepath():
    """
    :return: path given in the environment variable 'NRP_MODELS_DIRECTORY'
    """

    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        raise Exception("Server Error. NRP_MODELS_DIRECTORY not defined.")

    return path
