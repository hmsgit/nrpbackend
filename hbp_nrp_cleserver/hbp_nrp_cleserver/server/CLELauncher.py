"""
CLELauncher starts a new CLE. It substitutes the old cle_template.pyt
"""

__author__ = 'Lorenzo Vannuci, Georg Hinkel, Bernd Eckstein'

import os
import netifaces
import subprocess
import logging
import importlib
import argparse
import zipfile
import tempfile
import shutil

from RestrictedPython import compile_restricted

from geometry_msgs.msg import Pose

from hbp_nrp_cle.cle import config

from hbp_nrp_cleserver.bibi_config.notificator import Notificator, NotificatorHandler
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import compute_dependencies
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import \
    get_all_neurons_as_dict, generate_tf, import_referenced_python_tfs, correct_indentation

from hbp_nrp_commons.generated import bibi_api_gen
from hbp_nrp_commons.generated import exp_conf_api_gen

from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter
from hbp_nrp_cleserver.server.LocalGazebo import LocalGazeboBridgeInstance, \
    LocalGazeboServerInstance
from hbp_nrp_cleserver.server.LuganoVizClusterGazebo import LuganoVizClusterGazebo, XvfbXvnError
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

    def __init__(self, exd_conf, bibi_conf, experiment_path, gzserver_host, reservation, sim_id):
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
        """

        assert isinstance(exd_conf, exp_conf_api_gen.ExD_)
        assert isinstance(bibi_conf, bibi_api_gen.BIBIConfiguration)

        self.__exd_conf = exd_conf
        self.__bibi_conf = bibi_conf
        self.__experiment_path = experiment_path
        self.__gzserver_host = gzserver_host
        self.__reservation = reservation
        self.__sim_id = sim_id
        self.__abort_initialization = None
        self.__dependencies = compute_dependencies(bibi_conf)
        self.__gazebo_helper = None  # Will be instantiated after gazebo is started
        self.__tmp_robot_dir = None

        self.cle_server = None
        self.models_path = None
        self.gzweb = None
        self.gzserver = None

    def __handle_gazebo_shutdown(self):
        """
        Handles the case that Gazebo was shut down

        :param close_error: Any exception happened while closing Gazebo
        """
        logger.error("Gazebo died unexpectedly")
        if self.cle_server is not None and self.cle_server.lifecycle is not None:
            # Set the simulation to halted
            self.cle_server.lifecycle.failed()
            # If not already stopped, free simulation resources
            self.cle_server.lifecycle.stopped()
            # Avoid further notice
            self.gzserver.gazebo_died_callback = None
        # in case the simulation is still being started, we abort the initialization
        self.__abort_initialization = "Gazebo died unexpectedly"

    def __notify(self, message):
        """
        Checks whether the simulation should abort immediately
        """
        if self.__abort_initialization is not None:
            raise Exception("The simulation must abort due to: " + self.__abort_initialization)
        Notificator.notify(message, True)

    def cle_function_init(self, world_file, timeout=None, except_hook=None):
        """
        Initialize the Close Loop Engine. We still need the "world file" parameter
        in case the user is starting the experiment from the old web interface
        with the "Upload custom environment" button

        :param world_file: The environment (SDF) world file.
        :param timeout: The datetime object when the simulation timeouts or None
        :param except_hook: A handler method for critical exceptions
        """

        logger.info("Path is " + self.__experiment_path)
        os.chdir(self.__experiment_path)

        if self.__gzserver_host == 'local':
            self.gzserver = LocalGazeboServerInstance()
        elif self.__gzserver_host == 'lugano':
            self.gzserver = LuganoVizClusterGazebo(
                timeout.tzinfo if timeout is not None else None, self.__reservation
            )
        else:
            raise Exception("The gzserver location '{0}' is not supported.", self.__gzserver_host)

        # Create ROS server
        logger.info("Creating ROSCLEServer")
        self.cle_server = ROSCLEServer(self.__sim_id, timeout, self.gzserver)
        logger.info("Setting up backend Notificator")
        Notificator.register_notification_function(
            lambda subtask, update_progress:
            self.cle_server.notify_current_task(subtask, update_progress, True)
        )

        # We use the logger hbp_nrp_cle.user_notifications in the CLE to log
        # information that is useful to know for the user.
        # In here, we forward any info message sent to this logger to the notificator
        gazebo_logger = logging.getLogger('hbp_nrp_cle.user_notifications')
        gazebo_logger.setLevel(logging.INFO)
        gazebo_logger.handlers.append(NotificatorHandler())

        # Needed in order to cleanup global static variables
        nrp.start_new_tf_manager()

        # consts
        TIMESTEP = 0.02

        # set models path variable
        models_path = get_model_basepath()

        # Only for Frontend progress bar logic
        number_of_subtasks = 12
        if self.__bibi_conf.extRobotController is not None:
            number_of_subtasks = number_of_subtasks + 1
        if self.__bibi_conf.transferFunction is not None:
            number_of_subtasks = number_of_subtasks + 2 * len(self.__bibi_conf.transferFunction)

        self.cle_server.notify_start_task("Neurorobotics Closed Loop Engine",
                                          "Starting Neurorobotics Closed Loop Engine",
                                          number_of_subtasks=number_of_subtasks,
                                          block_ui=True)

        self.__notify("Starting Gazebo robotic simulator")  # subtask 1
        ifaddress = netifaces.ifaddresses(config.config.get('network', 'main-interface'))
        local_ip = ifaddress[netifaces.AF_INET][0]['addr']
        ros_master_uri = os.environ.get("ROS_MASTER_URI").replace('localhost', local_ip)

        self.gzserver.gazebo_died_callback = self.__handle_gazebo_shutdown

        # find robot
        robot_file = self.__bibi_conf.bodyModel
        logger.info("Robot: " + robot_file)
        robot_file_abs = self._get_robot_abs_path(robot_file)
        logger.info("RobotAbs: " + robot_file_abs)

        try:
            self.gzserver.start(ros_master_uri, self.__tmp_robot_dir)
        except XvfbXvnError as exception:
            logger.error(exception)
            error = "Recoverable error occurred. Please try again. Reason: {0}".format(
                    exception)
            raise Exception(error)

        self.__notify("Connecting to Gazebo robotic simulator")  # subtask 2
        self.__gazebo_helper = GazeboHelper()

        self.__notify("Starting Gazebo web client")  # subtask 3
        os.environ['GAZEBO_MASTER_URI'] = self.gzserver.gazebo_master_uri
        # We do not know here in which state the previous user did let us gzweb.
        self.gzweb = LocalGazeboBridgeInstance()
        self.gzweb.restart()

        self.__notify("Loading experiment environment")  # subtask 4
        self.__gazebo_helper.empty_gazebo_world()
        w_models, w_lights = self.__gazebo_helper.load_gazebo_world_file(world_file)

        # Create interfaces to Gazebo
        self.__notify("Loading robot")  # subtask 5

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

        robot_file = self.__bibi_conf.bodyModel
        logger.info("Robot: " + robot_file)
        if self.__is_collab_hack():
            robot_file_abs = os.path.join(self.__experiment_path, robot_file)
        else:
            robot_file_abs = os.path.join(models_path, robot_file)
        logger.info("RobotAbs: " + robot_file_abs)
        # spawn robot model
        self.__gazebo_helper \
            .load_gazebo_model_file('robot', robot_file_abs, rpose)

        # control adapter
        roscontrol = RosControlAdapter()
        # communication adapter
        roscomm = RosCommunicationAdapter()

        if self.__bibi_conf.extRobotController is not None:  # load external robot controller
            robot_controller_filepath = os.path.join(models_path,
                                                     self.__bibi_conf.extRobotController)
            if not os.path.isfile(robot_controller_filepath) and self.__tmp_robot_dir is not None:
                robot_controller_filepath = os.path.join(self.__tmp_robot_dir,
                                                         self.__bibi_conf.extRobotController)
            if os.path.isfile(robot_controller_filepath):
                self.__notify("Loading external robot controllers")  # +1
                res = subprocess.call([robot_controller_filepath, 'start'])
                if res > 0:
                    logger.error("The external robot controller could not be loaded")
                    self.shutdown()
                    return

        # Create interfaces to brain
        self.__notify("Loading Nest brain simulator")  # subtask 6
        # control adapter
        braincontrol = instantiate_control_adapter()
        # communication adapter
        braincomm = instantiate_communication_adapter()
        # Create transfer functions manager
        self.__notify("Connecting brain simulator to robot")  # subtask 7
        # tf manager
        tfmanager = nrp.config.active_node

        # set adapters
        tfmanager.robot_adapter = roscomm
        tfmanager.brain_adapter = braincomm

        # Import dependencies
        for dep in self.__dependencies:
            importlib.import_module(dep[:dep.rfind('.')])

        self.__notify("Loading brain and population configuration")  # subtask 8
        # load brain
        brainfilepath = self.__bibi_conf.brainModel.file
        if self.__is_collab_hack():
            if self.__experiment_path is not None:
                brainfilepath = os.path.join(self.__experiment_path, brainfilepath)
        else:
            brainfilepath = os.path.join(models_path, brainfilepath)
        neurons_config = get_all_neurons_as_dict(self.__bibi_conf.brainModel.populations)

        # initialize CLE
        self.__notify("Initializing CLE")  # subtask 9
        cle = ClosedLoopEngine(roscontrol, roscomm, braincontrol, braincomm, tfmanager, TIMESTEP)
        cle.initialize(brainfilepath, **neurons_config)

        # Set initial pose
        cle.initial_robot_pose = rpose
        # Set initial models and lights
        cle.initial_models = w_models
        cle.initial_lights = w_lights

        # Now that we have everything ready, we could prepare the simulation
        self.cle_server.prepare_simulation(cle, except_hook)

        self.__notify("Loading transfer functions")  # subtask 10

        # Create transfer functions
        import_referenced_python_tfs(self.__bibi_conf, self.__experiment_path)

        for i, tf in enumerate(self.__bibi_conf.transferFunction):
            self.__notify("Generating transfer function: %i" % (i + 1))  # subtask +1
            tf_code = generate_tf(tf, self.__bibi_conf)
            self.__notify("Loading transfer function: %s" % tf.name)  # named subtask +2
            tf_code = correct_indentation(tf_code, 0)
            tf_code = tf_code.strip() + "\n"
            logger.debug("TF: " + tf.name + "\n" + tf_code + '\n')

            try:
                new_code = compile_restricted(tf_code, '<string>', 'exec')
                nrp.set_transfer_function(tf_code, new_code, tf.name)
            # pylint: disable=broad-except
            except Exception as e:
                logger.error("Error while loading new transfer function")
                logger.error(e)
                raise

        # Wait for the backend rendering environment to load (for any sensors/cameras)
        self.__notify("Waiting for Gazebo simulated sensors to be ready")  # subtask 11
        self.__gazebo_helper.wait_for_backend_rendering()

        # Loading is completed.
        self.__notify("Finished")  # subtask 12
        self.cle_server.notify_finish_task()

        logger.info("CLELauncher Finished.")

        self.models_path = models_path

    def _get_robot_abs_path(self, robot_file):
        """
        Gets the absolute path of the given robot file

        :param robot_file: The robot file
        :return: the absolute path to the robot file
        """
        abs_file = os.path.join(self.__experiment_path, robot_file)
        name, ext = os.path.splitext(abs_file)
        ext = ext.lower()
        if ext == ".sdf":
            return abs_file
        elif ext == ".zip":
            name = os.path.split(name)[1] + "/model.sdf"
            with zipfile.ZipFile(abs_file) as robot_zip:
                try:
                    robot_zip.getinfo(name)
                except KeyError:
                    raise Exception("The robot zip archive must contain an sdf file named {0} "
                                    "at the root of the archive, but does not.".format(name))
                self.__tmp_robot_dir = tempfile.mkdtemp(suffix="robot")
                robot_zip.extractall(path=self.__tmp_robot_dir)
            return os.path.join(self.__tmp_robot_dir, name)

    # We try to shut down everything and aggregate the exceptions
    # pylint: disable=broad-except
    def shutdown(self):
        """
        Shutdown CLE
        """

        # Once we do reach this point, the simulation is stopped
        # and we can clean after ourselves.

        # Clean up gazebo after ourselves
        number_of_subtasks = 2
        if self.__bibi_conf.extRobotController is not None:
            number_of_subtasks = number_of_subtasks + 1
        try:
            try:
                self.cle_server.notify_start_task("Stopping simulation",
                                                  "Shutting down Gazebo robotic simulator",
                                                  number_of_subtasks=number_of_subtasks,
                                                  block_ui=False)
                notifications = True
            except Exception, e:
                logger.error("Could not send notifications")
                logger.exception(e)
                notifications = False

            # Do not empty Gazebo since Gazebo will be restarted anyhow
            if self.gzweb is not None:
                try:
                    self.gzweb.stop()
                except Exception, e:
                    logger.warning("gzweb could not be stopped successfully")
                    logger.exception(e)
            if self.gzserver is not None:
                try:
                    self.gzserver.stop()
                except Exception, e:
                    logger.warning("gzserver could not be stopped successfully")
                    logger.exception(e)

            # Stop any external robot controllers
            if self.__bibi_conf.extRobotController is not None:
                robot_controller_filepath = os.path.join(self.models_path,
                                                         self.__bibi_conf.extRobotController)
                if os.path.isfile(robot_controller_filepath):
                    if notifications:
                        self.cle_server.notify_current_task("Stopping external robot controllers",
                                                            update_progress=True, block_ui=False)
                    subprocess.check_call([robot_controller_filepath, 'stop'])

            # if the notification that shutdown was initiated did not work, there is no point
            # performing trying to notify the frontend here, either, since it is unlikely that this
            # works
            if notifications:
                # Shutdown CLE
                Notificator.register_notification_function(
                    lambda subtask, update_progress:
                    self.cle_server.notify_current_task(subtask, update_progress, False)
                )
                self.cle_server.notify_current_task("Shutting down Closed Loop Engine",
                                                    update_progress=True, block_ui=False)

                self.cle_server.notify_finish_task()
        finally:
            try:
                self.cle_server.shutdown()
            except Exception, e:
                logger.error("The cle server could not be shut down")
                logger.exception(e)

        # Cleanup ROS core nodes, services, and topics (the command should be almost
        # instant and exit, but wrap it in a timeout since it's semi-officially supported)
        logger.info("Cleaning up ROS nodes and services")
        os.system('rosnode kill /Watchdog /gazebo')
        os.system("echo 'y' | timeout -s SIGKILL 10s rosnode cleanup")

        # Delete temporary robot folder, if any
        if self.__tmp_robot_dir is not None:
            logger.info("Deleting temporary directory {temp}".format(temp=self.__tmp_robot_dir))
            shutil.rmtree(self.__tmp_robot_dir, ignore_errors=True)

    def __is_collab_hack(self):
        """
        This horrible hack is supposed to be dropped when we remove support for SDF cloning
        when we have introduced robot and env libraries.
        :return: true if we detect we are in collab models
        """
        return self.__experiment_path.startswith(tempfile.gettempdir())


def get_model_basepath():
    """
    :return: path given in the environment variable 'NRP_MODELS_DIRECTORY'
    """

    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        raise Exception("Server Error. NRP_MODELS_DIRECTORY not defined.")

    return path


def get_experiment_basepath():
    """
    :return: path given in the environment variable 'NRP_EXPERIMENTS_DIRECTORY'
    """

    path = os.environ.get('NRP_EXPERIMENTS_DIRECTORY')
    if path is None:
        raise Exception("Server Error. NRP_EXPERIMENTS_DIRECTORY not defined.")

    return path


if __name__ == '__main__':  # pragma: no cover
    # TODO: This should be separated into its own method such that we can unit test this code
    cle_launcher = None
    try:
        if os.environ["ROS_MASTER_URI"] == "":
            raise Exception("You should run ROS first.")

        parser = argparse.ArgumentParser()
        parser.add_argument('--exdconf', dest='exd_file',
                            help='specify the ExDConfiguration file', required=True)
        parser.add_argument('--environment', dest='environment_file',
                            help='specify the environment file', required=True)
        parser.add_argument('--experiment-path', dest='path',
                            help='specify the base experiment path', required=True)
        parser.add_argument('--gzserver-host', dest='gzserver_host',
                            help='the gzserver target host', required=True)
        parser.add_argument('--reservation', dest='reservation', default=None,
                            help='cluster resource reservation', required=False)
        parser.add_argument('--sim-id', dest='sim_id', type=int,
                            help='the simulation id to use', required=True)
        parser.add_argument('--timeout', dest='timeout',
                            help='the simulation default time allocated', required=True)

        music_parser = parser.add_mutually_exclusive_group(required=False)
        music_parser.add_argument('--music', dest='music', action='store_true',
                                  help='enable music support')
        music_parser.add_argument('--no-music', dest='music', action='store_false',
                                  help='explicitly disable music support (default)')
        parser.set_defaults(music=False)

        args = parser.parse_args()

        # expand any parameters (e.g. NRP_EXPERIMENTS_DIRECTORY) in paths
        args.exd_file = os.path.expandvars(args.exd_file)
        args.environment_file = os.path.expandvars(args.environment_file)
        args.path = os.path.expandvars(args.path)

        # music specific configuration for CLE
        if args.music:
            logger.info('Using MUSIC configuration and adapters for CLE')
            import music

            import hbp_nrp_cle.brainsim.config
            from hbp_nrp_music_interface.cle.MUSICPyNNCommunicationAdapter\
                import MUSICPyNNCommunicationAdapter
            from hbp_nrp_music_interface.cle.MUSICPyNNControlAdapter\
                import MUSICPyNNControlAdapter

            # exit code, 0 for success and -1 for any failures
            mpi_returncode = 0

            # write pid to lock file so launcher can always terminate us (failsafe)
            pid = os.getpid()
            with open('{}.lock'.format(pid), 'w') as pf:
                pf.write('{}'.format(pid))

            # initialize music and set the CLE to use MUSIC adapters
            music_setup = music.Setup()

            # TODO: the above calls to instantiate_{control/communication}_adapter
            #       should be generalized to allow us to specify the adapters and this
            #       backdoor specification should no longer be required
            hbp_nrp_cle.brainsim.config.communication_adapter_type = MUSICPyNNCommunicationAdapter
            hbp_nrp_cle.brainsim.config.control_adapter_type = MUSICPyNNControlAdapter

        # simplified launch process below from ROSCLESimulationFactory.py, avoid circular depdency
        # by importing here
        from hbp_nrp_cleserver.server.ROSCLESimulationFactory import get_experiment_data
        from hbp_nrp_cleserver.server.ROSCLESimulationFactory import \
            get_experiment_basepath as rcsf_get_experiment_basepath

        exd, bibi = get_experiment_data(args.exd_file)

        # parse the timeout string command line argument into a valid datetime
        import dateutil.parser as datetime_parser
        timeout_parsed = datetime_parser.parse(args.timeout.replace('_', ' '))

        # check the reservation argument, if empty default to None
        if args.reservation == '':
            args.reservation = None

        cle_launcher = CLELauncher(exd,
                                   bibi,
                                   rcsf_get_experiment_basepath(args.exd_file),
                                   args.gzserver_host, args.reservation, args.sim_id)
        cle_launcher.cle_function_init(args.environment_file, timeout_parsed)
        if cle_launcher.cle_server is None:
            raise Exception("Error in cle_function_init. Cannot start simulation.")

        # FIXME: This should be done more cleanly within the adapter, see [NRRPLT-4858]
        # the tag is a magic number to avoid circular build/release dependency for now but
        # this will be removed when the referenced bug is fixed
        # notify MPI/music processes that configuration is complete
        if args.music:
            from mpi4py import MPI
            for rank in xrange(MPI.COMM_WORLD.Get_size()):
                if rank != MPI.COMM_WORLD.Get_rank():
                    MPI.COMM_WORLD.send('ready', dest=rank, tag=100)
            MPI.COMM_WORLD.Barrier()

        logger.info('Starting CLE.')
        cle_launcher.cle_server.run()  # This is a blocking call, not to be confused with
                                       # threading.Thread.start

    except Exception as e:  # pylint: disable=broad-except

        # if running through MPI, catch Exception and terminate below to ensure brain processes
        # are also killed
        if args.music:
            logger.error('CLE aborted with message {}, terminating.'.format(e.message))
            print 'CLE aborted with message {}, terminating.'.format(e.message)  # if no logger
            logger.exception(e)
            mpi_returncode = -1

        # standalone non-music launch, propagate error
        else:
            raise

    finally:

        # always attempt to shutdown the CLE launcher and release resources
        if cle_launcher:
            logger.info('Shutting down CLE.')
            cle_launcher.shutdown()
            logger.info('Shutdown complete, terminating.')

    # terminate the MUSIC spawned brain processes
    # send a shutdown message in case the brain processes are in a recv loop at startup since they
    # seem to block and ignore the Abort command until receiving a message
    if args.music:
        from mpi4py import MPI
        for rank in xrange(MPI.COMM_WORLD.Get_size()):
            MPI.COMM_WORLD.isend('shutdown', dest=rank, tag=100)
        MPI.COMM_WORLD.Abort(mpi_returncode)
