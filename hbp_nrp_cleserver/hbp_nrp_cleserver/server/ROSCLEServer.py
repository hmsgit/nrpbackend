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
ROS wrapper around the CLE
"""

import json
import logging
import rospy
import numpy
import datetime
import time

from std_srvs.srv import Empty
from cle_ros_msgs.srv import SetBrainRequest

import textwrap
import re

from RestrictedPython import compile_restricted

# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of this CLE repository.
from cle_ros_msgs import srv
from cle_ros_msgs.msg import CLEError, ExperimentPopulationInfo
from cle_ros_msgs.msg import PopulationInfo, NeuronParameter, CSVRecordedFile
from hbp_nrp_cleserver.server import ROS_CLE_NODE_NAME, \
    SERVICE_SIM_RESET_ID, SERVICE_GET_TRANSFER_FUNCTIONS, SERVICE_EDIT_TRANSFER_FUNCTION, \
    SERVICE_ADD_TRANSFER_FUNCTION, SERVICE_DELETE_TRANSFER_FUNCTION, SERVICE_GET_BRAIN, \
    SERVICE_SET_BRAIN, SERVICE_GET_POPULATIONS, SERVICE_GET_CSV_RECORDERS_FILES, \
    SERVICE_CLEAN_CSV_RECORDERS_FILES, SERVICE_SIM_EXTEND_TIMEOUT_ID, \
    SERVICE_GET_STRUCTURED_TRANSFER_FUNCTIONS, SERVICE_SET_STRUCTURED_TRANSFER_FUNCTION
from hbp_nrp_cleserver.server import ros_handler
from hbp_nrp_cleserver.bibi_config import StructuredTransferFunction
from hbp_nrp_watchdog import Timer
import hbp_nrp_cle.tf_framework as tf_framework
from hbp_nrp_cle.tf_framework import TFLoadingException
import base64
from tempfile import NamedTemporaryFile
from hbp_nrp_cleserver.bibi_config.notificator import Notificator, NotificatorHandler
from hbp_nrp_cleserver.server.SimulationServerLifecycle import SimulationServerLifecycle
from hbp_nrp_commons.bibi_functions import find_changed_strings
import dateutil.parser as datetime_parser

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli, Georg Hinkel"
logger = logging.getLogger(__name__)

# We use the logger hbp_nrp_cle.user_notifications in the CLE to log
# information that is useful to know for the user.
# In here, we forward any info message sent to this logger to the notificator
gazebo_logger = logging.getLogger('hbp_nrp_cle.user_notifications')
gazebo_logger.setLevel(logging.INFO)
notificator_handler = NotificatorHandler()


def extract_line_number(tb):
    """
    Extracts the line number of the given traceback or returns -1

    :param tb: The traceback with the original error information
    """
    current = tb
    while current is not None:
        if current.tb_frame.f_code.co_filename == "<string>":
            return current.tb_lineno
        current = current.tb_next
    return -1


# pylint: disable=R0902
# the attributes are reasonable in this case
class ROSCLEServer(object):
    """
    A ROS server wrapper around the Closed Loop Engine.
    """
    STATUS_UPDATE_INTERVAL = 1.0

    def __init__(self, sim_id, timeout, gzserver, notificator):
        """
        Create the wrapper server

        :param sim_id: The simulation id
        :param timeout: The datetime when the simulation runs into a timeout
        :param gzserver: Gazebo simulator launch/control instance
        :param notificator: ROS state/error notificator interface
        """

        # ROS allows multiple calls to init_node, as long as the arguments are the same.
        # Allow multiple distributed processes to spawn nodes of the same name.
        rospy.init_node(ROS_CLE_NODE_NAME, anonymous=True)

        self.__lifecycle = None
        self.__cle = None

        self.__service_reset = None
        self.__service_extend_timeout = None
        self.__service_get_transfer_functions = None
        self.__service_add_transfer_function = None
        self.__service_edit_transfer_function = None
        self.__service_get_structured_transfer_functions = None
        self.__service_set_structured_transfer_function = None
        self.__service_delete_transfer_function = None
        self.__service_get_brain = None
        self.__service_set_brain = None
        self.__service_get_populations = None
        self.__service_get_CSV_recorders_files = None
        self.__service_clean_CSV_recorders_files = None

        self.__timer = Timer.Timer(ROSCLEServer.STATUS_UPDATE_INTERVAL,
                                   self.publish_state_update)
        self.__timeout = timeout
        self.__gzserver = gzserver
        self.__notificator = notificator

        self.__simulation_id = sim_id

        self._tuple2slice = (lambda x: slice(*x) if isinstance(x, tuple) else x)

    # pylint: disable=too-many-arguments
    def publish_error(self, source_type, error_type, message,
                      severity=CLEError.SEVERITY_ERROR, function_name="",
                      line_number=-1, offset=-1, line_text="", file_name=""):
        """
        Publishes an error and takes appropriate action if necessary

        :param source_type: The module the error message comes from, e.g. "Transfer Function"
        :param error_type: The error type, e.g. "Compile"
        :param message: The error message description, e.g. unexpected indent
        :param severity: The severity of the error
        :param function_name: The function name, if available
        :param line_number: The line number where the error occurred
        :param offset: The offset
        :param line_text: The text of the line causing the error
        :param file_name:
        """
        if severity >= CLEError.SEVERITY_ERROR:
            logger.exception("Error in {0} ({1}): {2}".format(source_type, error_type, message))
        self.__notificator.publish_error(
            CLEError(severity, source_type, error_type, message,
                     function_name, line_number, offset, line_text, file_name))
        if severity == CLEError.SEVERITY_CRITICAL and self.__lifecycle is not None:
            self.__lifecycle.failed()

    def __tf_except_hook(self, tf, tf_error, tb):
        """
        Handles an exception in the Transfer Functions

        :param tf: The transfer function that crashed
        :param tf_error: The exception that was thrown
        :param tb: The original traceback
        """
        if tf.updated:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Runtime", str(tf_error),
                               severity=CLEError.SEVERITY_ERROR, function_name=tf.name,
                               line_number=extract_line_number(tb))

    def prepare_simulation(self, cle, except_hook=None):
        """
        The CLE will be initialized within this method and ROS services for
        starting, pausing, stopping and resetting are setup here.

        :param cle: the closed loop engine
        :param except_hook: A handler method for critical exceptions
        """
        self.__cle = cle
        self.__lifecycle = SimulationServerLifecycle(self.__simulation_id, cle, self, except_hook)

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

        self.__service_extend_timeout = rospy.Service(
            SERVICE_SIM_EXTEND_TIMEOUT_ID(self.__simulation_id), srv.ExtendTimeout,
            self.__extend_timeout
        )

        self.__service_get_transfer_functions = rospy.Service(
            SERVICE_GET_TRANSFER_FUNCTIONS(self.__simulation_id), srv.GetTransferFunctions,
            self.__get_transfer_function_sources
        )

        self.__service_add_transfer_function = rospy.Service(
            SERVICE_ADD_TRANSFER_FUNCTION(self.__simulation_id), srv.AddTransferFunction,
            self.__add_transfer_function
        )

        self.__service_edit_transfer_function = rospy.Service(
            SERVICE_EDIT_TRANSFER_FUNCTION(self.__simulation_id), srv.EditTransferFunction,
            self.__edit_transfer_function
        )

        self.__service_get_structured_transfer_functions = rospy.Service(
            SERVICE_GET_STRUCTURED_TRANSFER_FUNCTIONS(self.__simulation_id),
            srv.GetStructuredTransferFunctions,
            self.__get_structured_transfer_functions
        )

        self.__service_set_structured_transfer_function = rospy.Service(
            SERVICE_SET_STRUCTURED_TRANSFER_FUNCTION(self.__simulation_id),
            srv.SetStructuredTransferFunction,
            self.__set_structured_transfer_function
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
            self.__try_set_brain
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

    @property
    def lifecycle(self):
        """
        Gets the lifecycle instance representing the current ROSCLEServer
        """
        return self.__lifecycle

    def __get_remaining(self):
        """
        Get the remaining time of the simulation
        """
        if self.__timeout is not None:
            # pylint: disable=E1103
            # false positive
            remaining = (self.__timeout - datetime.datetime.now(self.__timeout.tzinfo)) \
                .total_seconds()
            # pylint: enable=E1103
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
        return_val = srv.GetPopulationsResponse([PopulationInfo(str(p.name), str(p.celltype),
                                                                ROSCLEServer.__convert_parameters(
                                                                    p.parameters), p.gids,
                                                                p.indices)
                                                 for p in self.__cle.bca.get_populations()])
        return return_val

    # pylint: disable=unused-argument, no-self-use
    def __get_CSV_recorders_files(self, request):
        """
        Return the recorder file paths along with the name wanted by the user
        """
        return srv.GetCSVRecordersFilesResponse([CSVRecordedFile(recorded_file[0], recorded_file[1])
                                                 for recorded_file in
                                                 tf_framework.dump_csv_recorder_to_files()])

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
        return [NeuronParameter(str(key), float(parameters[key])) for key in parameters]

    # pylint: disable=unused-argument, no-self-use
    def __get_brain(self, request):
        """
        Returns the current neuronal network model. By default we do assume that if the sources are
        not available, the model comes from a h5 file. This has to be refined once we will be more
        confident in the fate of the h5 files.

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

        return [braintype, brain_code, data_type, json.dumps(tf_framework.get_brain_populations())]

    @staticmethod
    def change_transfer_function_for_population(change_population_mode, old_population_name,
                                                new_population_name, transfer_functions):
        """
        Modifies a single population name. Returns a value if needs user input.

        :param change_population_mode: The change population mode (as defined in SetBrainRequest)
        :param old_population_name: The old name of the population
        :param new_population_name: The new name of the population
        :param transfer_functions: The transfer functions to which the population change should be
                                   applied to.
        """
        for tf in transfer_functions:

            tfHasChanged = False

            for i in range(1, len(tf.params)):
                if hasattr(tf.params[i], 'spec'):
                    spec = tf.params[i].spec

                    if hasattr(spec, 'neurons'):
                        mapping = spec.neurons

                        if mapping is not None:
                            node = None
                            # required item is either neurons or its parent
                            if str(mapping.name) == old_population_name:
                                node = mapping
                            elif str(mapping.parent.name) == old_population_name:
                                node = mapping.parent
                            if node is not None:
                                if change_population_mode == \
                                        srv.SetBrainRequest.ASK_RENAME_POPULATION:
                                    # here we send a reply to the frontend to ask
                                    # the user a permission to change TFs
                                    return ["we ask the user if we change TFs", 0, 0, 1]
                                elif change_population_mode == \
                                        srv.SetBrainRequest.DO_RENAME_POPULATION:
                                    # permission granted, so we change TFs
                                    node.name = new_population_name
                                    tfHasChanged = True
            if tfHasChanged:
                source = tf.source
                pattern = re.compile(r'(?:(?<=\W)|^)(' + old_population_name + r')(?:(?=\W)|$)')
                modified_source = pattern.sub(new_population_name, source)
                if not modified_source == source:
                    tf.source = modified_source

    def change_transfer_functions(self, change_population, old_changed, new_added,
                                  transfer_functions):
        """
        Modifies population names. Returns a value if needs user input.

        :param change_population: The change population mode (as defined in SetBrainRequest)
        :param old_changed: A list of old population names
        :param new_added: A list of new population names
        :param transfer_functions: The transfer functions to which the changes should be applied
        """

        for changed_i in range(len(old_changed)):
            r = self.change_transfer_function_for_population(change_population,
                                                             str(old_changed[changed_i]),
                                                             str(new_added[changed_i]),
                                                             transfer_functions)
            if r is not None:
                return r

    def __try_set_brain(self, request):
        """
        Tries set the neuronal network according to the given request. If it fails, it restores
        the previous neuronal network.

        :param request: The mandatory rospy request parameter
        """
        previous_valid_brain = self.__get_brain(None)
        return_value = self.__set_brain(request.brain_populations, request.brain_type,
                                        request.brain_data, request.data_type,
                                        request.change_population)

        if return_value[0] != "":
            # failed to set new brain, we re set previous valid brain
            self.__set_brain(previous_valid_brain[3], previous_valid_brain[0],
                             previous_valid_brain[1], previous_valid_brain[2],
                             SetBrainRequest.ASK_RENAME_POPULATION)
        return return_value

    def __set_brain(self, brain_populations, brain_type, brain_data, data_type, change_population):
        """
        Sets the neuronal network according to the given parameters

        :param brain_populations: A dictionary indexed by population names and containing neuron
                                  indices. Neuron indices could be defined by individual integers,
                                  lists of integers or python slices. Python slices are defined by a
                                  dictionary containing the 'from', 'to' and 'step' values.
        :param brain_type: Type of the brain file ('h5' or 'py')
        :param brain_data: Contents of the brain file. Encoding given in field data_type
        :param data_type: Type of the brain_data field ('text' or 'base64')
        :param change_population: a flag to select an action on population name change, currently
                                  possible values are: 0 ask user for permission to replace;
                                  1 (permission granted) replace old name with a new one;
                                  2 proceed with no replace action
        """
        # pylint: disable-msg=R0914
        try:
            return_value = ["", 0, 0, 0]

            new_populations = [str(item) for item in json.loads(brain_populations).keys()]
            old_populations = [str(item) for item in tf_framework.get_brain_populations().keys()]

            old_changed = find_changed_strings(old_populations, new_populations)
            new_added = find_changed_strings(new_populations, old_populations)

            if len(new_added) == len(old_changed) >= 1:
                transfer_functions = tf_framework.get_transfer_functions()
                check_var_name_re = re.compile(r'^([a-zA-Z_]+\w*)$')
                for item in new_added:
                    if not check_var_name_re.match(item):
                        return ["Provided name \"" + item + "\" is not a valid population name",
                                0, 0, 0]
                r = self.change_transfer_functions(change_population, old_changed, new_added,
                                                   transfer_functions)
                if r is not None:
                    return r
            if self.__lifecycle.state != 'paused':
                # Member added by transitions library
                # pylint: disable=no-member
                self.__lifecycle.paused()
            with NamedTemporaryFile(prefix='brain', suffix='.' + brain_type, delete=False) as tmp:
                with tmp.file as brain_file:
                    if data_type == "text":
                        brain_file.write(brain_data)
                    elif data_type == "base64":
                        brain_file.write(base64.decodestring(brain_data))
                    else:
                        tmp.delete = True
                        return ["Data type {0} is invalid".format(data_type), 0, 0, 0]
                self.__cle.load_network_from_file(tmp.name, **json.loads(brain_populations))
        except ValueError, e:
            logger.exception(e)
            return_value = ["Population format is invalid: " + str(e), 0, 0, 0]
        except SyntaxError, e:
            logger.exception(e)
            return_value = ["The new brain could not be parsed: " + str(e), e.lineno, e.offset, 0]
        except Exception, e:
            logger.exception(e)
            return_value = ["Error changing neuronal network: " + str(e), 0, 0, 0]

        return return_value

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

    def __check_duplicate_tf_name(self, tf_name):
        """
        Internal helper to check whether a transfer function name already exists.

        :param tf_name: transfer function name to be checked.
        :return: Raises a ValueError if name already exists. Returns nothing otherwise.
        """

        tf_names = []
        for tf in self.__get_transfer_function_sources(None):
            tf_names.append(self.get_tf_name(tf)[1])

        if tf_name in tf_names:
            raise ValueError("Duplicate definition name")

    def __add_transfer_function(self, request):
        """
        Adds a new transfer function

        :param request: The ROS Service request message (cle_ros_msgs.srv.AddTransferFunction)
        :return: empty string for a successful compilation in restricted mode
                 (executed synchronously), an error message otherwise.
        """
        return self.__set_transfer_function(request, True)

    def __edit_transfer_function(self, request):
        """
        Modifies an existing transfer function

        :param request: The mandatory rospy request parameter
        :return: empty string for a successful compilation in restricted mode
                 (executed synchronously), an error message otherwise.
        """
        return self.__set_transfer_function(request, False)

    # pylint: disable=R0911
    def __set_transfer_function(self, request, new):
        """
        Patch a transfer function. This method is called when adding or editing a transfer function.

        :param request: The mandatory rospy request parameter
        :param new: A boolean indicating whether a transfer function is being added or modified.
        :return: empty string for a successful compilation in restricted mode
                 (executed synchronously), an error message otherwise.
        """
        new_source = textwrap.dedent(request.transfer_function_source)
        logger.info("About to compile transfer function with the following python code: \n"
                    + repr(new_source))

        tf_name = self.get_tf_name(new_source)
        if tf_name[0]:
            new_name = tf_name[1]
        else:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "NoOrMultipleNames",
                               tf_name[1], severity=CLEError.SEVERITY_ERROR)
            return tf_name[1]

        if not new:
            original_name = request.transfer_function_name
            # Delete synchronously the original if needed
            if original_name:
                tf_framework.delete_transfer_function(original_name)
        else:
            original_name = None

        # Make sure function name is not a duplicate
        try:
            self.__check_duplicate_tf_name(new_name)
        except ValueError:
            message = "duplicate Transfer Function name"
            # Select the correct function to highlight
            function_name = new_name if original_name is None else original_name
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Loading",
                               "Duplicate Definition Name", function_name=function_name)
            return message

        # Compile (synchronously) transfer function's new code in restricted mode
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

    @staticmethod
    def get_tf_name(source):
        """
        Check whether the function has a single definition name

        :param source: Transfer function source code
        :return: True and Function name if found, False and error message if none or multiple found
        """
        m = re.findall(r"(?:\n|^)def\s+(\w+)\s*\(", source)
        if len(m) != 1:
            if len(m) == 0:
                error_msg = "New Transfer Function has no definition name."
            else:
                error_msg = "New Transfer Function has multiple definition names."
            error_msg += " Compilation aborted"
            return False, error_msg
        return True, m[0]

    def __extend_timeout(self, request):
        """
        Extend the simulation timeout

        :param request: The new timeout
        :return: whether it could extend the timeout
        """
        new_timeout = datetime_parser.parse(request.timeout)

        if self.__gzserver is not None and not self.__gzserver.try_extend(new_timeout):
            return False

        self.__timeout = new_timeout

        return True

    # pylint: disable=unused-argument
    @staticmethod
    def __get_structured_transfer_functions(request):
        """
        Return the source code of the transfer functions

        :param request: The mandatory rospy request parameter
        """
        tfs = tf_framework.get_transfer_functions()
        return srv.GetStructuredTransferFunctionsResponse(
            [tf for tf in map(StructuredTransferFunction.extract_structure, tfs) if tf is not None]
        )

    def __set_structured_transfer_function(self, request):
        """
        Patch a transfer function

        :param request: The mandatory rospy request parameter
        :return: empty string for a successful compilation in restricted mode
                 (executed synchronously), an error message otherwise.
        """
        return self.__set_transfer_function(request.transfer_function.name,
                                            StructuredTransferFunction.
                                            generate_code_from_structured_tf(request.
                                                                             transfer_function))

    def __delete_transfer_function(self, request):
        """
        Delete an existing transfer function

        :param request: The mandatory rospy request parameter
        :return: True if the delete was successful, False if the transfer function does not exist.
        """
        self.__cle.stop()
        return tf_framework.delete_transfer_function(request.transfer_function_name)

    def publish_state_update(self):
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
            logger.debug(json.dumps(message))
            self.__notificator.publish_state(json.dumps(message))
        # pylint: disable=broad-except
        except Exception as e:
            logger.exception(e)

    def run(self):
        """
        This method blocks the caller until the simulation is finished
        """
        self.__lifecycle.done_event.wait()

        self.publish_state_update()
        time.sleep(1.0)
        logger.info("Finished main loop")

    def shutdown(self):
        """
        Shutdown the cle server
        """

        # the cle and services are initialized in prepare_simulation, which is not
        # guaranteed to have occurred before shutdown is called
        if self.__cle is not None:
            logger.info("Shutting down the closed loop service")
            self.__cle.shutdown()
            time.sleep(1)
            logger.info("Shutting down reset service")
            self.__service_reset.shutdown()
            logger.info("Shutting down extend timeout service")
            self.__service_extend_timeout.shutdown()
            logger.info("Shutting down get_transfer_functions service")
            self.__service_get_transfer_functions.shutdown()
            self.__service_get_structured_transfer_functions.shutdown()
            logger.info("Shutting down add_transfer_function service")
            self.__service_add_transfer_function.shutdown()
            logger.info("Shutting down set_transfer_function services")
            self.__service_edit_transfer_function.shutdown()
            self.__service_set_structured_transfer_function.shutdown()
            logger.info("Shutting down delete_transfer_function services")
            self.__service_delete_transfer_function.shutdown()
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
            time.sleep(1)

        # items initialized in the constructor
        self.__lifecycle = None
        self.__timer.cancel_all()

    def _reset_world(self, request):
        """
        Helper function for reset() call, it handles the RESET_WORLD message.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """

        with self.__notificator.task_notifier("Resetting Environment", "Emptying 3D world"):
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
            with self.__notificator.task_notifier("Resetting the simulation", ""):
                self.__notificator.update_task("Restoring the 3D world", False, True)
                self.__cle.reset_world(sdf_world_string)
                self.__notificator.update_task("Restoring the brain", False, True)
                self.__cle.reset_brain(brain_temp_path, network_conf_orig)
        else:
            with self.__notificator.task_notifier("Resetting the simulation", ""):
                self.__notificator.update_task("Restoring the 3D world", False, True)
                self.__cle.reset_world()
                self.__notificator.update_task("Restoring the brain", False, True)
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
            lambda subtask, update_progress: self.__notificator.update_task(subtask,
                                                                            update_progress,
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
