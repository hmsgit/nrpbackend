"""
ROSCLEServer unit test
"""

from cle_ros_msgs.srv import ResetSimulation, ResetSimulationRequest
from cle_ros_msgs.msg import CSVRecordedFile, ExperimentPopulationInfo
from hbp_nrp_cleserver.server import ROSCLEServer
import logging
from mock import patch, MagicMock, Mock, PropertyMock
from testfixtures import log_capture
from os import path
import base64
import unittest
import json
import threading
from functools import wraps
from multiprocessing import Process

from cle_ros_msgs import srv

__author__ = 'HBP NRP software team'


# Code for timeout decorator from
# http://stackoverflow.com/questions/14366761/
class TimeoutError(Exception):
    pass


def timeout(seconds=5, error_message="Timeout"):
    def decorator(func):
        def wrapper(*args, **kwargs):
            process = Process(None, func, None, args, kwargs)
            process.start()
            process.join(seconds)
            if process.is_alive():
                process.terminate()
                raise TimeoutError(error_message)

        return wraps(func)(wrapper)
    return decorator


class TestROSCLEServer(unittest.TestCase):

    LOGGER_NAME = ROSCLEServer.__name__

    def setUp(self):
        unittest.TestCase.setUp(self)

        # Mock the respective objects and make them available for all tests.
        # Also have a look at the following link:
        # https://docs.python.org/3.5/library/unittest.mock-examples.html#applying-the-same-patch-to-every-test-method
        cle_patcher = patch('hbp_nrp_cle.cle.CLEInterface.IClosedLoopControl')
        rospy_patcher = patch('hbp_nrp_cleserver.server.ROSCLEServer.rospy')

        # Ensure that the patchers are cleaned up correctly even in exceptional cases
        # e.g. when an exception was thrown.
        self.addCleanup(cle_patcher.stop)
        self.addCleanup(rospy_patcher.stop)

        self.__mocked_cle = cle_patcher.start()
        self.__mocked_cle.simulation_time = 0
        self.__mocked_cle.real_time = 0
        self.__mocked_cle.tf_elapsed_time = Mock(return_value=0)
        self.__mocked_cle.brainsim_elapsed_time = Mock(return_value=0)
        self.__mocked_cle.robotsim_elapsed_time = Mock(return_value=0)
        self.__mocked_rospy = rospy_patcher.start()

        with patch('hbp_nrp_cleserver.server.ROSCLEServer.threading') as threading_patch:
            # Set up our object under test and get sure it calls rospy.init in its
            # constructor.
            threading_patch.Thread = lambda target: target()
            self.__ros_cle_server = ROSCLEServer.ROSCLEServer(0)
            self.__ros_cle_server._ROSCLEServer__done_flag = Mock()

        # Be sure we create as many publishers as required.
        from hbp_nrp_cleserver.server import *
        number_of_publishers = 0
        for var_name in vars().keys():
            if (var_name.startswith("TOPIC_")):
                number_of_publishers += 1
        # Lifecycle topic not published by ROSCLEServer
        number_of_publishers -= 1

        self.assertEqual(number_of_publishers, self.__mocked_rospy.Publisher.call_count)

        # Assure that also the publish method of the rospy.Publisher is
        # injected as a mock here so that we can use it later in our single
        # test methods
        self.__mocked_ros_status_pub = ROSCLEServer.rospy.Publisher()
        self.__mocked_ros_status_pub.publish = MagicMock()
        self.__mocked_rospy.Publisher.return_value = self.__mocked_ros_status_pub

    def tearDown(self):
        # remove all handlers after each test!
        logging.getLogger(self.LOGGER_NAME).handlers = []

    def test_ros_node_initialized_with_right_name(self):
        self.__mocked_rospy.init_node.assert_called_with('ros_cle_simulation', anonymous=True)

    def test_prepare_initialization(self):
        self.__mocked_cle.is_initialized = False
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)
        self.assertEqual(12, self.__mocked_rospy.Service.call_count)

    def test_reset_simulation(self):
        self.__mocked_cle.is_initialized = False
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)
        self.__ros_cle_server._ROSCLEServer__lifecycle.state = 'paused'

        # Reset robot pose without Collab
        msg = ResetSimulationRequest()
        msg.reset_type = ResetSimulationRequest.RESET_ROBOT_POSE
        response, message = self.__ros_cle_server.reset_simulation(msg)
        self.assertEqual("", message)
        self.assertTrue(response)
        self.assertEqual(1, self.__mocked_cle.reset_robot_pose.call_count)
        self.__mocked_cle.reset_mock()

        # Reset Brain without Collab
        msg.reset_type = ResetSimulationRequest.RESET_BRAIN
        response, message = self.__ros_cle_server.reset_simulation(msg)
        self.assertEqual("", message)
        self.assertTrue(response)
        self.assertEqual(1, self.__mocked_cle.reset_brain.call_count)
        self.__mocked_cle.reset_mock()

        # Reset World without Collab
        msg.reset_type = ResetSimulationRequest.RESET_WORLD
        response, message = self.__ros_cle_server.reset_simulation(msg)
        self.assertEqual("", message)
        self.assertTrue(response)
        self.assertEqual(1, self.__mocked_cle.reset_world.call_count)
        self.__mocked_cle.reset_mock()

        # Reset Brain within Collab
        msg.reset_type = ResetSimulationRequest.RESET_BRAIN
        msg.brain_path = "/random/tmp/file.brain"
        msg.populations = [ExperimentPopulationInfo(name="population1", type=ExperimentPopulationInfo.TYPE_POPULATION_SLICE,
                                                    ids=[], start=0, stop=5, step=1),
                           ExperimentPopulationInfo(name="population2", type=ExperimentPopulationInfo.TYPE_POPULATION_SLICE,
                                                    ids=[], start=5, stop=10, step=1)]
        response, message = self.__ros_cle_server.reset_simulation(msg)
        self.assertEqual("", message)
        self.assertTrue(response)
        self.assertEqual(1, self.__mocked_cle.reset_brain.call_count)
        self.assertEqual(self.__mocked_cle.reset_brain.call_args_list[0][0][0],
                         "/random/tmp/file.brain")
        self.assertEqual(self.__mocked_cle.reset_brain.call_args_list[0][0][1],
                         {'population1': slice(0, 5, 1), 'population2': slice(5, 10, 1)})
        self.__mocked_cle.reset_mock()

        # Reset World within Collab
        msg.reset_type = ResetSimulationRequest.RESET_WORLD
        msg.world_sdf = "<a valid sdf string>"
        response, message = self.__ros_cle_server.reset_simulation(msg)
        self.assertEqual("", message)
        self.assertTrue(response)
        self.assertEqual(1, self.__mocked_cle.reset_world.call_count)
        self.assertEqual(self.__mocked_cle.reset_world.call_args_list[0][0][0], msg.world_sdf)
        self.__mocked_cle.reset_mock()

        # Reset Full
        msg.reset_type = ResetSimulationRequest.RESET_FULL
        msg.world_sdf = "<a valid sdf string>"
        msg.brain_path = "/random/tmp/file.brain"
        msg.populations = [ExperimentPopulationInfo(name="population1", type=ExperimentPopulationInfo.TYPE_POPULATION_SLICE,
                                                    ids=[], start=0, stop=5, step=1),
                           ExperimentPopulationInfo(name="population2", type=ExperimentPopulationInfo.TYPE_POPULATION_SLICE,
                                                    ids=[], start=5, stop=10, step=1)]
        response, message = self.__ros_cle_server.reset_simulation(msg)

    def __get_handlers_for_testing_main(self):
        self.__mocked_cle.is_initialized = True
        self.__ros_cle_server.prepare_simulation(self.__mocked_cle)

        # Get the ROS Service handlers; this will always be the third argument.
        arguments = {
            k.split('/')[-1]: v
            for (k, _, v) in
            [x[0] for x in self.__mocked_rospy.Service.call_args_list]
        }

        return arguments

    def test_get_brain(self):
        with patch("hbp_nrp_cleserver.server.ROSCLEServer.tf_framework") as mocked_tf_framework:
            mocked_tf_framework.get_brain_source.return_value = "Some python code"
            slice1 = { 'from': 1, 'to': 2, 'step': 3}
            slice2 = { 'from': 1, 'to': 2}
            populations_json_slice = {
              'population_1': 1, 'population_2': 2,
              'slice_1': slice1, 'slice_2': slice2,
              'list_1': [1, 2, 3]
            }
            mocked_tf_framework.get_brain_populations.return_value = populations_json_slice
            ros_callbacks = self.__get_handlers_for_testing_main()
            directory = path.split(__file__)[0]
            get_brain_implementation = ros_callbacks['get_brain']
            brain = get_brain_implementation(Mock())
            self.assertEqual("py", brain[0])
            self.assertEqual("Some python code", brain[1])
            self.assertEqual("text", brain[2])
            self.assertEqual(json.dumps(populations_json_slice), brain[3])
            mocked_tf_framework.get_brain_source.return_value = None
            brain = get_brain_implementation(Mock())
            self.assertEqual("h5", brain[0])
            self.assertEqual("base64", brain[2])

    @patch('hbp_nrp_cleserver.server.ROSCLEServer.NamedTemporaryFile')
    @patch('hbp_nrp_cleserver.server.ROSCLEServer.SimulationServerLifecycle')
    def test_set_brain(self, mock_tempfile, mock_lifecycle):

        mock_tempfile.name = 'random_name_tempfile'

        with patch("hbp_nrp_cleserver.server.ROSCLEServer.tf_framework") as mocked_tf_framework:
            slice1 = { 'from': 1, 'to': 2, 'step': 1}
            populations_json_slice = {
              'index1': 1,
              'slice_1': slice1,
              'list1': [1, 2, 3]
            }
            mocked_tf_framework.get_brain_populations.return_value = populations_json_slice

            ros_callbacks = self.__get_handlers_for_testing_main()
            self.__ros_cle_server._ROSCLEServer__lifecycle.state = 'started'
            self.__mocked_cle.network_file = PropertyMock()
            set_brain_implementation = ros_callbacks['set_brain']
            populations_erroneous = json.dumps({
                'index1': 1,
                'list1': [1, 2, 3],
                'slice1 new': {'from': 1, 'to': 2, 'step': 1}
            })
            request = Mock()
            request.data_type = "text"
            request.brain_type = "py"
            request.brain_data = "Dummy = None"
            request.brain_populations = populations_erroneous
            request.change_population = srv.SetBrainRequest.ASK_RENAME_POPULATION
            response = set_brain_implementation(request)
            self.assertNotEqual(response[0], "")

            populations_new = json.dumps({
                'index1': 1,
                'list1': [1, 2, 3],
                'slice1_new': {'from': 1, 'to': 2, 'step': 1}
            })
            request.change_population = srv.SetBrainRequest.DO_RENAME_POPULATION
            request.brain_populations = populations_new
            response = set_brain_implementation(request)
            self.assertEqual(response[0], "")

        self.__ros_cle_server._ROSCLEServer__lifecycle.paused.assert_called()
        expected_populations_arg = json.dumps(
            self.__mocked_cle.load_network_from_file.call_args[1]
        )
        self.assertEqual(populations_new, expected_populations_arg)
        self.__mocked_cle.load_network_from_file.reset_mock()
        request.data_type = "base64"
        request.brain_data = base64.encodestring(request.brain_data)
        response = set_brain_implementation(request)
        self.__mocked_cle.load_network_from_file.assert_called()
        request.data_type = "not_supported"
        response = set_brain_implementation(request)
        self.assertNotEqual("", response[0])

        request.brain_populations = "invalid JSON"
        response = set_brain_implementation(request)
        self.assertNotEqual("", response[0])
        request.brain_populations = populations_new

        def raise_syntax_error(foo):
            exec "Foo Bar"
        with patch("base64.decodestring") as mock_b64:
            request.data_type = "base64"
            mock_b64.side_effect = raise_syntax_error
            response = set_brain_implementation(request)
            self.assertNotEqual("", response[0])
            mock_b64.side_effect = Exception
            response = set_brain_implementation(request)
            self.assertNotEqual("", response[0])

    @patch('hbp_nrp_cleserver.server.ROSCLEServer.tf_framework')
    def test_CSV_recorders_functions(self, mocked_tf_framework):
        ros_callbacks = self.__get_handlers_for_testing_main()
        get_CSV_recorders_files_implementation = ros_callbacks['get_CSV_recorders_files']
        clean_CSV_recorders_files_implementation = ros_callbacks['clean_CSV_recorders_files']
        mocked_tf_framework.dump_csv_recorder_to_files = Mock()
        mocked_tf_framework.dump_csv_recorder_to_files.return_value = [('a','b'),('c','d')]
        result_value = get_CSV_recorders_files_implementation(None).files
        self.assertEqual(result_value[0].name, 'a')
        self.assertEqual(result_value[0].temporary_path, 'b')
        self.assertEqual(result_value[1].name, 'c')
        self.assertEqual(result_value[1].temporary_path, 'd')
        mocked_tf_framework.clean_csv_recorders_files = Mock()
        clean_CSV_recorders_files_implementation(None)
        self.assertEqual(1, mocked_tf_framework.clean_csv_recorders_files.call_count)

    @patch('hbp_nrp_cleserver.server.ROSCLEServer.tf_framework')
    def test_get_transfer_functions(self, mocked_tf_framework):
        tf = [MagicMock(), MagicMock()]
        tf[0].source = "tf_0 python code"
        tf[1].source = "tf_1 python code"
        mocked_tf_framework.get_transfer_functions = MagicMock(return_value=tf)
        ros_callbacks = self.__get_handlers_for_testing_main()
        transfer_functions_from_service = ros_callbacks['get_transfer_functions'](None)
        self.assertEqual(2, len(transfer_functions_from_service))
        self.assertEqual("tf_0 python code", transfer_functions_from_service[0])
        self.assertEqual("tf_1 python code", transfer_functions_from_service[1])

    @patch('hbp_nrp_cleserver.server.ROSCLEServer.tf_framework')
    def test_set_transfer_function(self, mocked_tf_framework):
        mocked_tf_framework.set_transfer_function = MagicMock(return_value=None)
        mocked_tf_framework.delete_transfer_function = MagicMock()
        ros_callbacks = self.__get_handlers_for_testing_main()
        set_transfer_function_handler = ros_callbacks['set_transfer_function']
        request = MagicMock()
        request.transfer_function_name = "tf_0"
        request.transfer_function_source = "def tf0(): \n return 0"
        response = set_transfer_function_handler(request)
        mocked_tf_framework.delete_transfer_function.assert_called_once_with("tf_0")
        self.assertEqual("", response)

        mocked_tf_framework.delete_transfer_function.reset_mock()
        request.transfer_function_name = ""
        request.transfer_function_source = "def tf0(): \n return 0"
        response = set_transfer_function_handler(request)
        self.assertEqual(0, mocked_tf_framework.delete_transfer_function.call_count)
        self.assertEqual("", response)

        request.transfer_function_name = "tf_1"
        request.transfer_function_source = "def (): \n return -1"
        response = set_transfer_function_handler(request)
        self.assertIn("no definition name", response)

        request.transfer_function_name = "tf_2"
        request.transfer_function_source = "def tf_2(): \n return -1 undefined"
        response = set_transfer_function_handler(request)
        self.assertIn("invalid syntax", response)
        self.assertIn("line 2", response)

        request.transfer_function_name = "tf_3"
        request.transfer_function_source = "def tf_3_a(): \n\treturn -1\ndef tf_3_b(): \n\treturn -2"
        response = set_transfer_function_handler(request)
        self.assertIn("multiple definition names", response)

        with patch('hbp_nrp_cleserver.server.ROSCLEServer.compile_restricted') as compile_restricted:
            request.transfer_function_name = "tf_4"
            request.transfer_function_source = "def tf_4(): \n return 0"
            compile_restricted.side_effect = Exception("foo")
            response = set_transfer_function_handler(request)
            self.assertEqual("foo", response)

        mocked_tf_framework.set_transfer_function.side_effect = Exception("bar")
        self.__ros_cle_server._ROSCLEServer__lifecycle = Mock()
        response = set_transfer_function_handler(request)
        self.assertEqual("bar", response)

        mocked_tf_framework.set_transfer_function.reset_mock()
        name ="tf_3_a"
        request.transfer_function_name = name
        request.transfer_function_source = "def tf_3_a(): \n\tdef tf_3_b(): \n\t\treturn -2\n\treturn -1"
        response = set_transfer_function_handler(request)
        assert name in mocked_tf_framework.set_transfer_function.call_args[0]
        self.assertNotIn("multiple definition names", response)

    @patch('hbp_nrp_cleserver.server.ROSCLEServer.tf_framework')
    def test_delete_transfer_function(self, mocked_tf_framework):
        mocked_tf_framework.delete_transfer_function = MagicMock(return_value=True)
        ros_callbacks = self.__get_handlers_for_testing_main()
        delete_transfer_function_handler = ros_callbacks['delete_transfer_function']
        request = MagicMock()
        request.transfer_function_name = "tf_0"
        response = delete_transfer_function_handler(request)
        self.assertEqual(True, response)

    def test_change_tf_for_population(self):
        mock_tf = MagicMock()
        mock_tf.params = []
        mock_tf.source = "tf_source node_name"
        tfs = [mock_tf]
        ROSCLEServer.ROSCLEServer.change_transfer_function_for_population(MagicMock(),MagicMock(), MagicMock(), tfs, MagicMock())
        # check nothing has changed
        self.assertEqual(mock_tf.source, "tf_source node_name")

        mock_mapping = MagicMock()
        mock_mapping.name = "node_name"

        mock_spec = MagicMock(spec=["neurons"])
        mock_spec.neurons = mock_mapping

        mock_params = MagicMock(spec=["spec"])
        mock_params.spec = mock_spec
        mock_tf.params = ["t", mock_params] # for some reason the real data has a t as in the list

        old_changed = ["node_name"]
        tfs = [mock_tf]
        self.assertEqual(ROSCLEServer.ROSCLEServer.change_transfer_function_for_population(srv.SetBrainRequest.ASK_RENAME_POPULATION, old_changed, MagicMock(), tfs, 0), ["we ask the user if we change TFs", 0, 0, 1])
        self.assertEqual(mock_tf.source, "tf_source node_name")

        mock_new_added = ["new_node_name"]
        mock_mapping.name = ""
        mock_parent = MagicMock()
        mock_parent.name = "node_name"
        mock_mapping.parent = mock_parent

        ROSCLEServer.ROSCLEServer.change_transfer_function_for_population(srv.SetBrainRequest.DO_RENAME_POPULATION, old_changed, mock_new_added, tfs, 0)
        self.assertEqual(mock_parent.name, mock_new_added[0])
        self.assertEqual(mock_tf.source, "tf_source new_node_name")

    def test_rospy_spin_is_called(self):
        # Assert that rospy.spin has been called when the roscleserver has been created
        self.assertEqual(1, self.__mocked_rospy.spin.call_count)

    def test_notify_start_task(self):
        task_name = 'test_name'
        subtask_name = 'test_subtaskname'
        number_of_subtasks = 1
        block_ui = False
        self.__ros_cle_server.notify_start_task(
            task_name, subtask_name, number_of_subtasks, block_ui)
        self.assertEqual(1, self.__mocked_ros_status_pub.publish.call_count)
        message = {'progress': {'task': task_name,
                                'subtask': subtask_name,
                                'number_of_subtasks': number_of_subtasks,
                                'subtask_index': 0,
                                'block_ui': block_ui}}
        self.__mocked_ros_status_pub.publish.assert_called_with(
            json.dumps(message))

    def test_task(self):
        mock_publisher = Mock()
        self.__ros_cle_server._ROSCLEServer__ros_status_pub = mock_publisher
        self.__ros_cle_server.notify_start_task('task', 'subtask', 1, False)
        self.assertEquals(mock_publisher.publish.call_count, 1)
        self.__ros_cle_server.notify_current_task('new_subtask', True, False)
        self.assertEquals(mock_publisher.publish.call_count, 2)
        self.__ros_cle_server.notify_finish_task()
        self.assertEquals(mock_publisher.publish.call_count, 3)

    def test_shutdown(self):
        z = self.__ros_cle_server._ROSCLEServer__cle = MagicMock()
        a = self.__ros_cle_server._ROSCLEServer__service_reset  = MagicMock()
        b = self.__ros_cle_server._ROSCLEServer__current_task = None
        c = self.__ros_cle_server._ROSCLEServer__service_get_transfer_functions = MagicMock()
        d = self.__ros_cle_server._ROSCLEServer__service_extend_timeout = MagicMock()
        e = self.__ros_cle_server._ROSCLEServer__service_set_transfer_function = MagicMock()
        f = self.__ros_cle_server._ROSCLEServer__service_get_brain = MagicMock()
        g = self.__ros_cle_server._ROSCLEServer__service_set_brain = MagicMock()
        h = self.__ros_cle_server._ROSCLEServer__service_get_populations = MagicMock()
        i = self.__ros_cle_server._ROSCLEServer__service_get_CSV_recorders_files = MagicMock()
        j = self.__ros_cle_server._ROSCLEServer__service_clean_CSV_recorders_files = MagicMock()
        k = self.__ros_cle_server._ROSCLEServer__service_get_structured_transfer_functions = MagicMock()
        l = self.__ros_cle_server._ROSCLEServer__service_set_structured_transfer_function = MagicMock()
        m = self.__ros_cle_server._ROSCLEServer__service_delete_transfer_function = MagicMock()
        n = self.__ros_cle_server._ROSCLEServer__ros_cle_error_pub = MagicMock()
        o = self.__ros_cle_server._ROSCLEServer__ros_status_pub = MagicMock()

        self.__ros_cle_server.shutdown()
        for x in [a, c, d, e, f, g, h, i, j, k, l, m, z]:
            self.assertEquals(x.shutdown.call_count, 1)
        self.assertEquals(n.unregister.call_count, 1)
        self.assertEquals(o.unregister.call_count, 1)

    @log_capture(level=logging.WARNING)
    def test_notify_current_task(self, logcapture):
        self.__ros_cle_server.notify_current_task("new_subtask", True, True)
        logcapture.check(
            (self.LOGGER_NAME, 'WARNING', "Can't update a non existing task.")
        )

    @log_capture(level=logging.WARNING)
    def test_notify_finish_task_no_task(self, logcapture):
        self.__ros_cle_server.notify_finish_task()
        logcapture.check(
            (self.LOGGER_NAME, 'WARNING', "Can't finish a non existing task.")
        )


if __name__ == '__main__':
    unittest.main()
