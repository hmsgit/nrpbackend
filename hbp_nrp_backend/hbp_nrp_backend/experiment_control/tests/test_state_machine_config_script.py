__author__ = 'Sebastian Krach'

from hbp_nrp_backend.experiment_control.state_machine_configuration_script \
    import initialize_state_machines, request_sm_termination, start_state_machines, wait_sm_termination
import unittest
import mock
import os


class TestStateMachineConfigurationScript(unittest.TestCase):

    def setUp(self):
        self.__sm_mock1 = mock.Mock()
        self.__sm_mock1.is_running.return_value = False
        self.__sm_mock2 = mock.Mock()
        self.__sm_mock2.is_running.return_value = False
        self.__sm_mock3 = mock.Mock()
        self.__sm_mock3.is_running.return_value = False
        self.__mock_dict = {'test1': self.__sm_mock1, 'test2': self.__sm_mock2, 'test3': self.__sm_mock3}

    def test_initialize(self):
        with mock.patch("hbp_nrp_backend.experiment_control.ExperimentStateMachineInstance.__init__") as esmi_mock:
            with mock.patch("hbp_nrp_backend.experiment_control.ExperimentStateMachineInstance.initialize_sm")\
                    as esmi_initialize_mock:
                esmi_mock.return_value = None
                directory = os.path.split(__file__)[0]
                sm_path = os.path.join(directory, 'state_machine_mock.py')
                state_machines = initialize_state_machines({'test-sm': sm_path})
                self.assertEquals(len(state_machines), 1)
                self.assertTrue("test-sm" in state_machines)
                esmi_initialize_mock.assert_called_once_with()

                state_machines = initialize_state_machines({})
                self.assertEquals(len(state_machines), 0)

    def test_start_state_machines(self):
        start_state_machines(self.__mock_dict)
        self.assertEquals(self.__sm_mock1.start_execution.call_count, 1)
        self.assertEquals(self.__sm_mock2.start_execution.call_count, 1)
        self.assertEquals(self.__sm_mock3.start_execution.call_count, 1)

    def test_sm_termination(self):
        request_sm_termination(self.__mock_dict)
        self.assertEquals(self.__sm_mock1.request_termination.call_count, 1)
        self.assertEquals(self.__sm_mock2.request_termination.call_count, 1)
        self.assertEquals(self.__sm_mock3.request_termination.call_count, 1)

    def test_sm_wait_termination(self):
        wait_sm_termination(self.__mock_dict)
        self.assertEquals(self.__sm_mock1.wait_termination.call_count, 1)
        self.assertEquals(self.__sm_mock2.wait_termination.call_count, 1)
        self.assertEquals(self.__sm_mock3.wait_termination.call_count, 1)

if __name__ == '__main__':
    unittest.main()
