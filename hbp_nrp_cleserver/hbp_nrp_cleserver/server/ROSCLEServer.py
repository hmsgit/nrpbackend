"""
ROS wrapper around the CLE
"""

import json
import logging
import threading
import rospy
import numpy
import datetime

from std_msgs.msg import String
from std_srvs.srv import Empty

import textwrap
import re

from RestrictedPython import compile_restricted

# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of this CLE repository.
from cle_ros_msgs import srv
from cle_ros_msgs.msg import CLEError, ExperimentPopulationInfo
from cle_ros_msgs.msg import PopulationInfo, NeuronParameter, CSVRecordedFile
from hbp_nrp_cleserver.server import ROS_CLE_NODE_NAME, \
    TOPIC_STATUS, TOPIC_CLE_ERROR, SERVICE_SIM_RESET_ID, \
    SERVICE_GET_TRANSFER_FUNCTIONS, SERVICE_SET_TRANSFER_FUNCTION, \
    SERVICE_DELETE_TRANSFER_FUNCTION, SERVICE_GET_BRAIN, SERVICE_SET_BRAIN, \
    SERVICE_GET_POPULATIONS, SERVICE_GET_CSV_RECORDERS_FILES, \
    SERVICE_CLEAN_CSV_RECORDERS_FILES
from hbp_nrp_cleserver.server import ros_handler, Timer
import hbp_nrp_cle.tf_framework as tf_framework
from hbp_nrp_cle.tf_framework import TFLoadingException
import base64
from tempfile import NamedTemporaryFile
from hbp_nrp_cleserver.bibi_config.notificator import Notificator, NotificatorHandler
import contextlib
from hbp_nrp_cleserver.server.SimulationServerLifecycle import SimulationServerLifecycle

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli, Georg Hinkel"
logger = logging.getLogger(__name__)


# We use the logger hbp_nrp_cle.user_notifications in the CLE to log
# information that is useful to know for the user.
# In here, we forward any info message sent to this logger to the notificator
gazebo_logger = logging.getLogger('hbp_nrp_cle.user_notifications')
gazebo_logger.setLevel(logging.INFO)
notificator_handler = NotificatorHandler()


# pylint: disable=R0902
# the attributes are reasonable in this case
class ROSCLEServer(object):
    """
    A ROS server wrapper around the Closed Loop Engine.
    """
    STATUS_UPDATE_INTERVAL = 1.0

    def __init__(self, sim_id, timeout=None):
        """
        Create the wrapper server

        :param sim_id: The simulation id
        :param timeout: The datetime when the simulation runs into a timeout
        """

        # ROS allows multiple calls to init_node, as long as the arguments are the same.
        # Allow multiple distributed processes to spawn nodes of the same name.
        rospy.init_node(ROS_CLE_NODE_NAME, anonymous=True)

        self.__lifecycle = None
        self.__cle = None

        self.__service_reset = None
        self.__service_get_transfer_functions = None
        self.__service_set_transfer_function = None
        self.__service_delete_transfer_function = None
        self.__service_get_brain = None
        self.__service_set_brain = None
        self.__service_get_populations = None
        self.__service_get_CSV_recorders_files = None
        self.__service_clean_CSV_recorders_files = None

        self.__timer = Timer.Timer(ROSCLEServer.STATUS_UPDATE_INTERVAL,
                                   self.__publish_state_update)
        self.__timeout = timeout

        self.__simulation_id = sim_id
        self.__ros_status_pub = rospy.Publisher(
            TOPIC_STATUS,
            String,
            queue_size=10  # Not expecting more that 10hz
        )
        self.__ros_cle_error_pub = rospy.Publisher(
            TOPIC_CLE_ERROR,
            CLEError,
            queue_size=10  # Not expecting more that 10hz
        )

        self.__current_task = None
        self.__current_subtask_count = 0
        self.__current_subtask_index = 0

        self._tuple2slice = (lambda x: slice(*x) if isinstance(x, tuple) else x)

        # Start another thread calling rospy.spin
        # This line has been put here because it has to be called before prepare_simulation,
        # where all the 'rospy.Service's are initialized.
        # This is the best place I managed to found, as it doesn't seem to hurt anybody.
        # Any enhancement is warmly welcomed.
        rospy_thread = threading.Thread(target=rospy.spin)
        rospy_thread.setDaemon(True)
        rospy_thread.start()

    # pylint: disable=too-many-arguments
    def publish_error(self, source_type, error_type, message,
                      severity=CLEError.SEVERITY_ERROR, function_name="",
                      line_number=-1, offset=-1, line_text="", file_name=""):
        """
        Publishes an error message

        :param severity: The severity of the error
        :param source_type: The module the error message comes from, e.g. "Transfer Function"
        :param error_type: The error type, e.g. "Compile"
        :param message: The error message description, e.g. unexpected indent
        :param function_name: The function name, if available
        :param line_number: The line number where the error ocurred
        :param offset: The offset
        :param line_text: The text of the line causing the error
        """
        if severity >= CLEError.SEVERITY_ERROR:
            logger.exception("Error in {0} ({1}): {2}".format(source_type, error_type, message))
        self.__ros_cle_error_pub.publish(
            CLEError(severity, source_type, error_type, message,
                     function_name, line_number, offset, line_text, file_name))
        if severity == CLEError.SEVERITY_CRITICAL and self.__lifecycle is not None:
            self.__lifecycle.failed()

    def __tf_except_hook(self, tf, tf_error):
        """
        Handles an exception in the Transfer Functions

        :param tf: The transfer function that crashed
        :param tf_error: The exception that was thrown
        """
        if tf.updated:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Runtime", str(tf_error),
                               severity=CLEError.SEVERITY_ERROR, function_name=tf.name)

    def prepare_simulation(self, cle):
        """
        The CLE will be initialized within this method and ROS services for
        starting, pausing, stopping and resetting are setup here.

        :param cle: the closed loop engine
        """
        self.__cle = cle
        self.__lifecycle = SimulationServerLifecycle(self.__simulation_id, cle, self)

        logger.info("Registering ROS Service handlers")

        # We have to use lambdas here (!) because otherwise we bind to the state which is in place
        # during the time we set the callback! I.e. we would bind directly to the initial state.
        # The x parameter is defined because of the architecture of rospy.
        # rospy is expecting to have handlers which takes two arguments (self and x). The
        # second one holds all the arguments sent through ROS (defined in the srv file).
        # Even when there is no input argument for the service, rospy requires this.

        # pylint: disable=unnecessary-lambda

        self.__service_reset = rospy.Service(
            SERVICE_SIM_RESET_ID(self.__simulation_id), srv.ResetSimulation,
            lambda x: self.reset_simulation(x)
        )

        self.__service_get_transfer_functions = rospy.Service(
            SERVICE_GET_TRANSFER_FUNCTIONS(self.__simulation_id), srv.GetTransferFunctions,
            self.__get_transfer_function_sources
        )

        self.__service_set_transfer_function = rospy.Service(
            SERVICE_SET_TRANSFER_FUNCTION(self.__simulation_id), srv.SetTransferFunction,
            self.__set_transfer_function
        )

        self.__service_delete_transfer_function = rospy.Service(
            SERVICE_DELETE_TRANSFER_FUNCTION(self.__simulation_id), srv.DeleteTransferFunction,
            self.__delete_transfer_function
        )

        self.__service_get_brain = rospy.Service(
            SERVICE_GET_BRAIN(self.__simulation_id), srv.GetBrain,
            self.__get_brain
        )

        self.__service_set_brain = rospy.Service(
            SERVICE_SET_BRAIN(self.__simulation_id), srv.SetBrain,
            self.__set_brain
        )

        self.__service_get_populations = rospy.Service(
            SERVICE_GET_POPULATIONS(self.__simulation_id), srv.GetPopulations,
            self.__get_populations
        )

        self.__service_get_CSV_recorders_files = rospy.Service(
            SERVICE_GET_CSV_RECORDERS_FILES(self.__simulation_id), srv.GetCSVRecordersFiles,
            self.__get_CSV_recorders_files
        )

        self.__service_clean_CSV_recorders_files = rospy.Service(
            SERVICE_CLEAN_CSV_RECORDERS_FILES(self.__simulation_id), Empty,
            self.__clean_CSV_recorders_files
        )

        tf_framework.TransferFunction.excepthook = self.__tf_except_hook
        self.__timer.start()

    def __get_remaining(self):
        """
        Get the remaining time of the simulation
        """
        if self.__timeout is not None:
            remaining = (self.__timeout - datetime.datetime.now(self.__timeout.tzinfo)) \
                .total_seconds()
            if remaining < 0:
                self.__lifecycle.stopped()
            return max(0, int(remaining))
        else:
            return 0

    # pylint: disable=unused-argument
    def __get_populations(self, request):
        """
        Gets the populations available in the neural network
        """
        return_val = srv.GetPopulationsResponse([
            PopulationInfo(str(p.name), str(p.celltype),
                           ROSCLEServer.__convert_parameters(p.parameters), p.gids)
            for p in self.__cle.bca.get_populations()
        ])
        return return_val

    # pylint: disable=unused-argument, no-self-use
    def __get_CSV_recorders_files(self, request):
        """
        Return the recorder file paths along with the name wanted by the user
        """
        return srv.GetCSVRecordersFilesResponse([
            CSVRecordedFile(recorded_file[0], recorded_file[1])
            for recorded_file in tf_framework.dump_csv_recorder_to_files()
        ])

    # pylint: disable=unused-argument, no-self-use
    @ros_handler
    def __clean_CSV_recorders_files(self, request):
        """
        Remove temporary recorders files from the server
        """
        tf_framework.clean_csv_recorders_files()

    @staticmethod
    def __convert_parameters(parameters):
        """
        Converts the given parameters to ROS types

        :param parameters: A parameter dictionary
        :return: A list of ROS-compatible neuron parameters
        """
        return [
            NeuronParameter(str(key), float(parameters[key]))
            for key in parameters
        ]

    # pylint: disable=unused-argument, no-self-use
    def __get_brain(self, request):
        """
        Returns the current neuronal network model. By default we
        do assume that if the sources are not available, the model
        comes from a h5 file. This has to be refined once we will
        be more confident in the fate of the h5 files.

        :param request: The rospy request parameter
        :return: an array compatible with the GetBrain.srv ROS service
        """
        braintype = "h5"
        data_type = "base64"
        brain_code = "N/A"
        if tf_framework.get_brain_source():
            braintype = "py"
            data_type = "text"
            brain_code = tf_framework.get_brain_source()

        return [
            braintype,
            brain_code,
            data_type,
            json.dumps(tf_framework.get_brain_populations())
        ]

    @staticmethod
    def findChangedStrings(list_a, list_b):
        """
        Returns a list that contains changed population names,
        i.e. stings from list_a are searched in list_b, not found strings are
        returned

        :param list_a: looked up strings to be found in list_b
        :param list_b: looked up strings are compared against strings in list_b
        :return: a list of strings
        """
        changed = []
        for element_a in list_a:
            element_a = str(element_a)
            found = False
            for element_b in list_b:
                element_b = str(element_b)
                if element_a == element_b:
                    found = True
                    break
            if not found:
                changed.append(element_a)
        return changed

    def check_population_names(self, request):
        """
        Extracts and returns population names that were changed,
        plus new population names from request
        :return: tuple with first existing names list,
        then new names list, i.e. [old_changed, new_added]
        """
        old_popls = []
        existingPopls = tf_framework.get_brain_populations()
        for p in existingPopls:
            old_popls.append(str(p))

        new_popls = []
        for popl in json.loads(request.brain_populations):
            new_popls.append(str(popl))

        old_changed = self.findChangedStrings(old_popls, new_popls)
        new_added = self.findChangedStrings(new_popls, old_popls)

        return [old_changed, new_added]

    @staticmethod
    def change_transfer_functions(request, old_changed, new_added, transferFunctions):
        """
        Modifies population names. Returns a value if needs user input.
        """
        pattern = re.compile(r'(?:(?<=\W)|^)(' + str(old_changed[0]) + r')(?:(?=\W)|$)')
        for tf in transferFunctions:

            tfHasChanged = False

            for i in range(1, len(tf.params)):
                if hasattr(tf.params[i], 'spec'):
                    spec = tf.params[i].spec

                    if hasattr(spec, 'neurons'):
                        mapping = spec.neurons

                        if mapping is not None:
                            node = None
                            # required item is either neurons or its parent
                            if str(mapping.name) == str(old_changed[0]):
                                node = mapping
                            elif str(mapping.parent.name) == str(old_changed[0]):
                                node = mapping.parent
                            if node:
                                if request.change_population == \
                                        srv.SetBrainRequest.ASK_RENAME_POPULATION:
                                    # here we send a reply to the frontend to ask
                                    # the user a permission to change TFs
                                    return ["we ask the user if we change TFs", 0, 0, 1]
                                elif request.change_population == \
                                        srv.SetBrainRequest.DO_RENAME_POPULATION:
                                    # permission granted, so we change TFs
                                    node.name = new_added[0]
                                    tfHasChanged = True
            if tfHasChanged:
                source = tf.source
                modified_source = pattern.sub(str(new_added[0]), source)
                if not modified_source == source:
                    tf.source = modified_source

    def __set_brain(self, request):
        """
        Sets the neuronal network according to the given request

        :param request: The mandatory rospy request parameter
        """
        try:
            [old_changed, new_added] = self.check_population_names(request)
            returnValue = ["", 0, 0, 0]

            if (len(old_changed) == 1) and (len(new_added) == 1):
                transferFunctions = tf_framework.get_transfer_functions()
                checkVarNameRe = re.compile(r'^([a-zA-Z_]+\w*)$')
                match = checkVarNameRe.match(str(new_added[0]))
                if not match:
                    return ["Provided name \"" + str(new_added[0]) +
                            "\" is not a valid population name", 0, 0, 0]
                r = self.change_transfer_functions(request, old_changed, new_added,
                                                   transferFunctions)
                if r is not None:
                    return r
            if self.__lifecycle.state != 'paused':
                # Member added by transitions library
                # pylint: disable=no-member
                self.__lifecycle.paused()
            with NamedTemporaryFile(prefix='brain', suffix='.' +
                                    request.brain_type, delete=False) as tmp:
                with tmp.file as brain_file:
                    if request.data_type == "text":
                        brain_file.write(request.brain_data)
                    elif request.data_type == "base64":
                        brain_file.write(base64.decodestring(request.brain_data))
                    else:
                        tmp.delete = True
                        return ["Data type {0} is invalid".format(request.data_type), 0, 0, 0]
                self.__cle.load_network_from_file(
                    tmp.name, **json.loads(request.brain_populations))
        except ValueError, e:
            returnValue = ["Population format is invalid: " + str(e), 0, 0, 0]
        except SyntaxError, e:
            returnValue = ["The new brain could not be parsed: " + str(e), e.lineno, e.offset, 0]
        except Exception, e:
            returnValue = ["Error changing neuronal network: " + str(e), 0, 0, 0]

        return returnValue

    # pylint: disable=unused-argument
    @staticmethod
    def __get_transfer_function_sources(request):
        """
        Return the source code of the transfer functions

        :param request: The mandatory rospy request parameter
        """
        tfs = tf_framework.get_transfer_functions()
        arr = numpy.asarray([tf.source.encode('UTF-8') for tf in tfs])
        return arr

    def __set_transfer_function(self, request):
        """
        Patch a transfer function

        :param request: The mandatory rospy request parameter
        :return: empty string for a successful compilation in restricted mode
                (executed synchronously),
                 an error message otherwise.
        """

        original_name = request.transfer_function_name
        # Delete synchronously the original if needed
        if original_name:
            tf_framework.delete_transfer_function(original_name)

        # Update transfer function's source code
        new_source = textwrap.dedent(request.transfer_function_source)

        # Check whether the function has a single definition name
        logger.info(
            "About to compile transfer function originally named "
            + original_name + "\n"
            + "with the following python code: \n"
            + repr(new_source)
        )
        m = re.findall(r"(?:\n|^)def\s+(\w+)\s*\(", new_source)
        if len(m) != 1:
            error_msg = original_name
            if len(m) == 0:
                error_msg += " has no definition name."
            else:
                error_msg += " has multiple definition names."
            error_msg += " Compilation aborted"
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "NoOrMultipleNames",
                               error_msg,
                               severity=CLEError.SEVERITY_ERROR, function_name=original_name)
            return error_msg

        # Compile (synchronously) transfer function's new code in restricted mode
        new_name = m[0]
        new_code = None
        try:
            new_code = compile_restricted(new_source, '<string>', 'exec')
        except SyntaxError as e:
            message = "Syntax Error while compiling the updated" \
                + " transfer function named " + new_name \
                + " in restricted mode.\n" \
                + str(e)
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Compile", str(e),
                               severity=CLEError.SEVERITY_ERROR, function_name=new_name,
                               line_number=e.lineno, offset=e.offset, line_text=e.text,
                               file_name=e.filename)
            return message
        except Exception as e:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Compile", str(e),
                               severity=CLEError.SEVERITY_ERROR, function_name=new_name)
            return e.message

        # Make sure CLE is stopped. If already stopped, these calls are harmless.
        # (Execution of updated code is asynchronous)
        self.__cle.stop()
        try:
            tf_framework.set_transfer_function(new_source, new_code, new_name)
        except TFLoadingException as e:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Loading", e.message,
                               severity=CLEError.SEVERITY_ERROR, function_name=e.tf_name)
            return e.message
        except Exception as e:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Loading", e.message,
                               severity=CLEError.SEVERITY_CRITICAL, function_name=new_name)
            return e.message
        return ""

    def __delete_transfer_function(self, request):
        """
        Delete an existing transfer function

        :param request: The mandatory rospy request parameter
        :return: always True as this command is executed asynchronously.
                 ROS forces us to return a value.
        """
        self.__cle.stop()
        tf_framework.delete_transfer_function(request.transfer_function_name)

        return True

    def __publish_state_update(self):
        """
        Publish the state and the remaining timeout
        """
        try:
            if self.__lifecycle is None:
                logger.warn("Trying to publish state even though no simulation is active")
                return
            message = {
                'state': str(self.__lifecycle.state),
                'timeout': self.__get_remaining(),
                'simulationTime': int(self.__cle.simulation_time),
                'realTime': int(self.__cle.real_time),
                'transferFunctionsElapsedTime': self.__cle.tf_elapsed_time(),
                'brainsimElapsedTime': self.__cle.brainsim_elapsed_time(),
                'robotsimElapsedTime': self.__cle.robotsim_elapsed_time()
            }
            logger.info(json.dumps(message))
            self.__ros_status_pub.publish(json.dumps(message))
        # pylint: disable=broad-except
        except Exception as e:
            logger.exception(e)

    def run(self):
        """
        This method blocks the caller until the simulation is finished
        """
        self.__lifecycle.done_event.wait()

        self.__publish_state_update()
        logger.info("Finished main loop")

    def shutdown(self):
        """
        Shutdown the cle server
        """
        logger.info("Shutting down the closed loop service")
        self.__cle.shutdown()
        logger.info("Shutting down reset service")
        self.__service_reset.shutdown()
        logger.info("Shutting down get_transfer_functions service")
        self.__service_get_transfer_functions.shutdown()
        logger.info("Shutting down set_transfer_function service")
        self.__service_set_transfer_function.shutdown()
        logger.info("Shutting down get_brain service")
        self.__service_get_brain.shutdown()
        logger.info("Shutting down set_brain service")
        self.__service_set_brain.shutdown()
        logger.info("Shutting down get_populations service")
        self.__service_get_populations.shutdown()
        logger.info("Shutting down get_CSV_recorders_files service")
        self.__service_get_CSV_recorders_files.shutdown()
        logger.info("Shutting down clean_CSV_recorders_files service")
        self.__service_clean_CSV_recorders_files.shutdown()
        logger.info("Unregister error/transfer_function topic")
        self.__ros_cle_error_pub.unregister()
        logger.info("Unregister status topic")
        self.__ros_status_pub.unregister()
        self.__lifecycle = None
        self.__timer.cancel_all()

    def notify_start_task(self, task_name, subtask_name, number_of_subtasks, block_ui):
        """
        Sends a status notification that a task starts on the ROS status topic.
        This method will save the task name and the task size in class members so that
        it could be reused in subsequent call to the notify_current_task method.

        :param: task_name: Title of the task (example: initializing experiment).
        :param: subtask_name: Title of the first subtask. Could be empty
                (example: loading Virtual Room).
        :param: number_of_subtasks: Number of expected subsequent calls to
                notify_current_task(_, True, _).
        :param: block_ui: Indicate that the client should block any user interaction.
        """
        if self.__current_task is not None:
            logger.warn(
                "Previous task was not closed properly, closing it now.")
            self.notify_finish_task()
        self.__current_task = task_name
        self.__current_subtask_count = number_of_subtasks
        message = {'progress': {'task': task_name,
                                'subtask': subtask_name,
                                'number_of_subtasks': number_of_subtasks,
                                'subtask_index': self.__current_subtask_index,
                                'block_ui': block_ui}}
        self.__ros_status_pub.publish(json.dumps(message))

    def notify_current_task(self, new_subtask_name, update_progress, block_ui):
        """
        Sends a status notification that the current task is updated with a new subtask.

        :param: subtask_name: Title of the first subtask. Could be empty
                (example: loading Virtual Room).
        :param: update_progress: Boolean indicating if the index of the current subtask
                should be updated (usually yes).
        :param: block_ui: Indicate that the client should block any user interaction.
        """
        if self.__current_task is None:
            logger.warn("Can't update a non existing task.")
            return
        if update_progress:
            self.__current_subtask_index += 1
        message = {'progress': {'task': self.__current_task,
                                'subtask': new_subtask_name,
                                'number_of_subtasks': self.__current_subtask_count,
                                'subtask_index': self.__current_subtask_index,
                                'block_ui': block_ui}}
        self.__ros_status_pub.publish(json.dumps(message))

    def notify_finish_task(self):
        """
        Sends a status notification that the current task is finished.
        """
        if self.__current_task is None:
            logger.warn("Can't finish a non existing task.")
            return
        message = {'progress': {'task': self.__current_task,
                                'done': True}}
        self.__ros_status_pub.publish(json.dumps(message))
        self.__current_subtask_count = 0
        self.__current_subtask_index = 0
        self.__current_task = None

    def _reset_world(self, request):
        """
        Helper function for reset() call, it handles the RESET_WORLD message.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """

        with self.task_notifier("Resetting Environment", "Emptying 3D world"):
            if request.world_sdf is not None and request.world_sdf is not "":
                sdf_world_string = request.world_sdf
                self.__cle.reset_world(sdf_world_string)
            else:
                self.__cle.reset_world()

    def _reset_brain(self, request):
        """
        Helper function for reset() call, it handles the RESET_BRAIN message.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """

        if request.brain_path is not None and request.brain_path is not "":
            brain_temp_path = request.brain_path
            neurons_conf = request.populations
            network_conf_orig = {
                p.name: self._get_population_value(p) for p in neurons_conf
            }
            self.__cle.reset_brain(brain_temp_path, network_conf_orig)
        else:
            self.__cle.reset_brain()

    def _reset_full(self, request):
        """
        Helper function for reset() call, it handles the RESET_FULL message.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """

        self.__cle.reset_robot_pose()
        if request.world_sdf is not None and request.world_sdf is not "":
            sdf_world_string = request.world_sdf
            brain_temp_path = request.brain_path
            neurons_conf = request.populations
            network_conf_orig = {
                p.name: self._get_population_value(p) for p in neurons_conf
            }
            with self.task_notifier("Resetting the simulation", ""):
                self.notify_current_task("Restoring the 3D world", False, True)
                self.__cle.reset_world(sdf_world_string)
                self.notify_current_task("Restoring the brain", False, True)
                self.__cle.reset_brain(brain_temp_path, network_conf_orig)
        else:
            with self.task_notifier("Resetting the simulation", ""):
                self.notify_current_task("Restoring the 3D world", False, True)
                self.__cle.reset_world()
                self.notify_current_task("Restoring the brain", False, True)
                self.__cle.reset_brain()

        # Member added by transitions library
        # pylint: disable=no-member
        self.__lifecycle.initialized()

    def start_fetching_gazebo_logs(self):
        """
        Starts to fetch the logs from gazebo and putting them as notifications
        """
        gazebo_logger.handlers.append(notificator_handler)
        Notificator.register_notification_function(
            lambda subtask, update_progress: self.notify_current_task(subtask, update_progress,
                                                                      True)
        )

    def stop_fetching_gazebo_logs(self):
        """
        Stop fetching logs from gazebo and putting them as notifications
        """
        gazebo_logger.handlers.remove(notificator_handler)

    # pylint: disable=broad-except
    def reset_simulation(self, request):
        """
        Handler for the CLE reset() call.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """

        rsr = srv.ResetSimulationRequest
        reset_type = request.reset_type

        if reset_type == rsr.RESET_OLD:
            # Member added by transitions library
            # pylint: disable=no-member
            self.__lifecycle.initialized()
        else:
            try:
                self.start_fetching_gazebo_logs()
                if reset_type == rsr.RESET_ROBOT_POSE:
                    self.__cle.reset_robot_pose()
                elif reset_type == rsr.RESET_WORLD:
                    self._reset_world(request)
                elif reset_type == rsr.RESET_BRAIN:
                    self._reset_brain(request)
                elif reset_type == rsr.RESET_FULL:
                    self._reset_full(request)
            except Exception as e:
                return False, str(e)
            finally:
                self.stop_fetching_gazebo_logs()

        return True, ""

    @contextlib.contextmanager
    def task_notifier(self, task_name, subtask_name):

        """
        Task notifier context manager

        :param task_name:
        :param subtask_name:
        """

        self.notify_start_task(task_name, subtask_name,
                               number_of_subtasks=0,
                               block_ui=True)
        try:
            yield
        finally:
            self.notify_finish_task()

    @staticmethod
    def _get_population_value(population_info):
        """
        Retrieves the population index from the given population info

        :param population_info: The serialized population info object
        :return: The index that the population info represents
        """
        if population_info.type == ExperimentPopulationInfo.TYPE_ENTIRE_POPULATION:
            return None
        elif population_info.type == ExperimentPopulationInfo.TYPE_POPULATION_SLICE:
            return slice(population_info.start, population_info.stop, population_info.step)
        elif population_info.type == ExperimentPopulationInfo.TYPE_POPULATION_LISTVIEW:
            return population_info.ids
