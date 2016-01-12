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
from hbp_nrp_cle.robotsim.GazeboHelper import GazeboHelper

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

    def __init__(self, exd_conf, bibi_conf, experiment_path, gzserver_host, sim_id):
        """
        Constructor of CLELauncher

        @param exd_conf: Experiment Configuration
        @type exd_conf: generated_experiment_api.ExD
        @param bibi_conf: Bibi Configuration
        @type bibi_conf: generated_bibi_api.BIBIConfiguration
        @param experiment_path: path for the experiment. Useful because sometime
        we are starting a template (with path being /opt/hbp/...) and sometime we
        are starting an experiment from a collab temp folder (with path being
        /tmp/...)
        @param gzserver_host: 'local' or 'lugano'
        @type gzserver_host: str
        @param sim_id: simulation id
        @type sim_id: int
        @return: [cle_server, models_path, gzweb, gzserver]
        """

        assert isinstance(exd_conf, exp_conf_api_gen.ExD_)
        assert isinstance(bibi_conf, bibi_api_gen.BIBIConfiguration)

        self.__exd_conf = exd_conf
        self.__bibi_conf = bibi_conf
        self.__experiment_path = experiment_path
        self.__gzserver_host = gzserver_host
        self.__sim_id = sim_id
        self.__dependencies = compute_dependencies(bibi_conf)
        self.__gazebo_helper = None  # Will be instantiated after gazebo is started

    def cle_function_init(self, world_file):
        """
        Initialize the Close Loop Engine. We still need the "world file" parameter
        in case the user is starting the experiment from the old web interface
        with the "Upload custom environment" button

        @param world_file: The environment (SDF) world file.
        @return: [cle_server, models_path, gzweb, gzserver] (all the initialized sub-components)
        """

        logger.info("Path is " + self.__experiment_path)

        # Create ROS server
        logger.info("Creating ROSCLEServer")
        cle_server = ROSCLEServer(self.__sim_id)
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
        if self.__bibi_conf.extRobotController is not None:
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
        models_path = get_experiment_basepath()

        Notificator.notify("Resetting Gazebo robotic simulator", True)  # subtask 1

        ifaddress = netifaces.ifaddresses(config.config.get('network', 'main-interface'))
        local_ip = ifaddress[netifaces.AF_INET][0]['addr']
        ros_master_uri = os.environ.get("ROS_MASTER_URI").replace('localhost', local_ip)

        gzweb = LocalGazeboBridgeInstance()

        gzserver = None
        if self.__gzserver_host == 'local':
            gzserver = LocalGazeboServerInstance()
            gzserver.start(ros_master_uri)
        elif self.__gzserver_host == 'lugano':
            gzserver = LuganoVizClusterGazebo()
            gzserver.start(ros_master_uri)

        self.__gazebo_helper = GazeboHelper()

        if gzserver is None:
            logger.error("No configuration found for gzserver_host: '{0}'"
                         "".format(self.__gzserver_host))
            self.shutdown(cle_server, None, None, None)
            return [None, None, None, None]

        os.environ['GAZEBO_MASTER_URI'] = gzserver.gazebo_master_uri
        # We do not know here in which state the previous user did let us gzweb.
        gzweb.restart()

        self.__gazebo_helper.empty_gazebo_world()

        Notificator.notify("Loading experiment environment", True)  # subtask 2

        self.__gazebo_helper.load_gazebo_world_file(world_file)

        # Create interfaces to Gazebo
        Notificator.notify("Loading neuRobot", True)  # subtask 3

        robot_initial_pose = self.__exd_conf.environmentModel.robotPose
        if robot_initial_pose is not None:
            rpose = Pose()
            rpose.position.x = robot_initial_pose.x
            rpose.position.y = robot_initial_pose.y
            rpose.position.z = robot_initial_pose.z
            rpose.orientation.x = robot_initial_pose.ux
            rpose.orientation.y = robot_initial_pose.uy
            rpose.orientation.z = robot_initial_pose.uz
            rpose.orientation.w = robot_initial_pose.theta
        else:
            rpose = None

        # spawn robot model
        self.__gazebo_helper \
            .load_gazebo_model_file('robot', self.__bibi_conf.bodyModel, rpose)

        # control adapter
        roscontrol = RosControlAdapter()
        # communication adapter
        roscomm = RosCommunicationAdapter()

        if self.__bibi_conf.extRobotController is not None:  # load external robot controller
            robot_controller_filepath = os.path.join(models_path,
                                                     self.__bibi_conf.extRobotController)
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
        for dep in self.__dependencies:
            importlib.import_module(dep[:dep.rfind('.')])

        # Create CLE
        cle = ClosedLoopEngine(roscontrol, roscomm, braincontrol, braincomm, tfmanager, TIMESTEP)

        Notificator.notify("Loading Brain", True)  # subtask 6
        # load brain
        brainfilepath = self.__bibi_conf.brainModel.file
        if self.__experiment_path is not None:
            brainfilepath = os.path.join(self.__experiment_path, brainfilepath)

        # initialize everything
        Notificator.notify("Initializing CLE", True)  # subtask 7
        neurons_config = get_all_neurons_as_dict(self.__bibi_conf.brainModel.populations)
        cle.initialize(brainfilepath, **neurons_config)

        # Set initial pose
        cle.initial_robot_pose = rpose

        # Now that we have everything ready, we could prepare the simulation
        cle_server.prepare_simulation(cle, self.__exd_conf.timeout)

        Notificator.notify("Injecting Transfer Functions", True)  # subtask 8

        # Create transfer functions
        import_referenced_python_tfs(self.__bibi_conf, self.__experiment_path)

        for tf in self.__bibi_conf.transferFunction:
            tf_code = generate_tf(tf, self.__bibi_conf)
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
                raise

        # Loading is completed.
        Notificator.notify("Finished", True)  # subtask 9
        cle_server.notify_finish_task()

        logger.info("CLELauncher Finished.")

        return [cle_server, models_path, gzweb, gzserver]

    def shutdown(self, cle_server, models_path, gzweb, gzserver):
        """
        Shutdown CLE

        @param cle_server: the cle_server
        @param models_path: the models path
        @param gzweb: gzweb
        @param gzserver: gzserver
        """

        # Once we do reach this point, the simulation is stopped and we could clean after ourselves.
        # Clean up gazebo after ourselves
        cle_server.notify_start_task("Stopping simulation",
                                     "Emptying 3D world",
                                     number_of_subtasks=2,
                                     block_ui=False)
        self.__gazebo_helper.empty_gazebo_world()

        if gzweb is not None:
            gzweb.stop()
        if gzserver is not None:
            gzserver.stop()

        if self.__bibi_conf.extRobotController is not None:  # optionally stop all external robot
            # controllers
            robot_controller_filepath = os.path.join(models_path,
                                                     self.__bibi_conf.extRobotController)
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


def get_experiment_basepath():
    """
    :return: path given in the environment variable 'NRP_MODELS_DIRECTORY'
    """

    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        raise Exception("Server Error. NRP_MODELS_DIRECTORY not defined.")

    return path
