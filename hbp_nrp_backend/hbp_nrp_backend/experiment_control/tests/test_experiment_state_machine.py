__author__ = 'Sebastian Krach'

from hbp_nrp_backend.experiment_control import ExperimentStateMachineInstance, \
    ConcurrentExecutionException, TimeoutException, MAX_TIMEOUT
from hbp_nrp_backend.exd_config.default_state_machine import DefaultStateMachine
from testfixtures import LogCapture
import unittest
import threading
import rospy
import mock
import time
import os


class TestExperimentStateMachine(unittest.TestCase):
    def setUp(self):
        self.__start_tread_event = threading.Event()
        self.__block_tread_event = threading.Event()

    def tearDown(self):
        self.__start_tread_event.clear()

    def __initialize_side_effect(self, *args, **kwargs):
        self.assertTrue(len(args), 1)
        self.__initialize_cb = args[0]

    def __start_execution_side_effect(self, *args, **kwargs):
        self.__sm_thread = threading.current_thread()
        self.__start_tread_event.set()
        self.__block_tread_event.wait()

    def test_create_instance(self):
        instance = ExperimentStateMachineInstance("test_id")
        self.assertEquals(instance.sm_id, "test_id")
        self.assertIsNone(instance.sm_instance)
        self.assertFalse(instance.is_running)

        # instance = ExperimentStateMachineInstance("test_id", "print 'test'")
        self.assertRaises(AttributeError, ExperimentStateMachineInstance, "test_id", "print 'test'")

    def test_initialize_state_machine(self):
        directory = os.path.split(__file__)[0]

        instance = ExperimentStateMachineInstance("test_id")
        self.assertRaises(AttributeError, instance.initialize_sm)

        sm_path = os.path.join(directory, 'state_machine_mock.py')
        with open(sm_path, "r") as sm_source:
            instance = ExperimentStateMachineInstance("test_id", sm_source.read())

        instance.initialize_sm()
        self.assertIsInstance(instance.sm_instance, mock.Mock)

        self.assertTrue(callable(instance.sm_module.initialize_cb))

    def __init_test_class(self):
        directory = os.path.split(__file__)[0]
        sm_path = os.path.join(directory, 'state_machine_mock.py')
        with open(sm_path, "r") as sm_source:
            self.__test_class = ExperimentStateMachineInstance("test_id", sm_source.read())

        self.__test_class.initialize_sm()
        self.__sm_mock = self.__test_class.sm_instance
        self.__initialize_cb = self.__test_class.sm_module.initialize_cb

    def __start_test_class(self):
        self.__sm_mock.execute.side_effect = self.__start_execution_side_effect

        self.__test_class.start_execution()
        self.__start_tread_event.wait()

    def test_execution_state_machine(self):
        self.__init_test_class()
        self.__start_test_class()
        time.sleep(0.1)
        self.assertNotEquals(threading.current_thread, self.__sm_thread)

        self.assertIsNone(self.__test_class.result)
        self.__initialize_cb(None, [], "FINISHED")
        self.__block_tread_event.set()
        self.__test_class.wait_termination()
        self.assertEquals(self.__test_class.result, "FINISHED")

    def test_invalid_actions_while_execution(self):
        self.__init_test_class()
        self.__start_test_class()
        time.sleep(0.1)

        directory = os.path.split(__file__)[0]
        sm_path = os.path.join(directory, 'state_machine_mock.py')
        with open(sm_path, "r") as sm_source:
            self.assertRaises(ConcurrentExecutionException,
                              setattr, self.__test_class, "sm_source", sm_source.read())

        self.assertRaises(ConcurrentExecutionException, self.__test_class.initialize_sm)

        self.assertRaises(ConcurrentExecutionException, self.__test_class.start_execution)

        self.__initialize_cb(None, [], "FINISHED")
        self.__block_tread_event.set()

        self.__sm_thread.join()
        self.assertFalse(self.__sm_thread.is_alive())

        sm_module_old = self.__test_class.sm_module
        sm_path = os.path.join(directory, 'state_machine_mock_duplicate.py')
        with open(sm_path, "r") as sm_source_duplicate:
            self.__test_class.sm_source = sm_source_duplicate.read()

        self.assertNotEquals(sm_module_old, self.__test_class.sm_module)

        self.assertNotEquals(self.__test_class.sm_instance, self.__sm_mock)

    def test_preemption_state_machine(self):
        self.__init_test_class()
        self.__start_test_class()
        time.sleep(0.1)

        self.assertTrue(self.__test_class.is_running)
        self.assertFalse(self.__sm_mock.sm.request_preempt.called)
        self.__test_class.request_termination()
        self.assertTrue(self.__test_class.is_running)
        self.__sm_mock.sm.request_preempt.assert_called_once_with()

        self.__initialize_cb(None, [], "FINISHED")
        self.__block_tread_event.set()
        self.__sm_thread.join()
        self.assertFalse(self.__test_class.is_running)

    def test_reset_state_machine(self):
        self.__init_test_class()
        self.__start_test_class()
        time.sleep(0.1)

        self.assertTrue(self.__test_class.is_running)
        self.assertFalse(self.__sm_mock.sm.request_preempt.called)
        self.__test_class.request_termination()
        self.assertTrue(self.__test_class.is_running)
        self.__sm_mock.sm.request_preempt.assert_called_once_with()

        self.__initialize_cb(None, [], "FINISHED")
        self.__block_tread_event.set()
        self.__sm_thread.join()
        self.assertFalse(self.__test_class.is_running)

    def test_error_cases(self):
        self.__test_class = ExperimentStateMachineInstance("start-without-initialize")
        self.assertRaises(AttributeError,
                          setattr, self.__test_class, "sm_source",
                          "print 'This should lead to an error!'")

        self.assertRaises(AttributeError, self.__test_class.start_execution)
        self.assertRaises(AttributeError, self.__test_class.wait_termination)
        self.assertRaises(AttributeError, self.__test_class.request_termination)

    def test_thread_timeout(self):
        self.__init_test_class()
        self.__start_test_class()

        self.__test_class.request_termination()
        self.__initialize_cb(None, [], "FINISHED")

        self.assertRaises(TimeoutException, self.__test_class.wait_termination, 0.5)

        self.__block_tread_event.set()
        self.__sm_thread.join()
        self.assertFalse(self.__test_class.is_running)

        l = LogCapture()
        self.__test_class.wait_termination(2 * MAX_TIMEOUT)
        l.check(('hbp_nrp_backend.experiment_control.__ExperimentStateMachine',
                 'WARNING',
                 'Timeout not a valid number or greater than allowed maximum. Set to maximum.'))
        l.uninstall()

    def test_semaphore_timeout(self):
        self.__init_test_class()
        self.__start_test_class()

        self.__test_class.request_termination()
        self.__block_tread_event.set()

        self.assertRaises(TimeoutException, self.__test_class.wait_termination, 0.5)

        self.__initialize_cb(None, [], "FINISHED")

        self.__sm_thread.join()
        self.assertFalse(self.__test_class.is_running)


if __name__ == '__main__':
    unittest.main()
