"""
Test file for testing hbp_nrp_backend.simulation_control.transitions
"""
import rospy

__author__ = 'Alessandro Ambrosano'

import unittest
import mock
from rospy import ROSException
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
from hbp_nrp_backend.simulation_control import simulations, Simulation, transitions
from hbp_nrp_backend.rest_server.__SimulationControl import UserAuthentication


class TestTransition(unittest.TestCase):
    """
    Class for testing hbp_nrp_backend.simulation_control.transitions
    """

    def setUp(self):
        self.__roscleclient_mock = mock.patch('hbp_nrp_backend.cle_interface.ROSCLEClient.ROSCLEClient').start()
        self.__roscleclient_mock.start = mock.Mock()
        self.__roscleclient_mock.pause = mock.Mock()
        self.__roscleclient_mock.stop = mock.Mock()
        self.__roscleclient_mock.reset = mock.Mock()
        self.__sm_mock = mock.Mock()
        self.__sm_mock.is_running.return_value = False

        transitions.generate_bibi = mock.Mock(return_value=mock.Mock())
        transitions.initialize_experiment = mock.Mock(return_value="set_cle")
        transitions.generate_experiment_control = mock.Mock(return_value={'test_sm': 'test_sm.py'})
        transitions.initialize_state_machines = mock.Mock(return_value={'test_sm': self.__sm_mock})

        patch_CollabClient = mock.patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

        del simulations[:]
        simulations.append(
            Simulation(0, 'ExDConf/ExDXMLExample.xml', None, 'local', 'default-owner', state='created'))
        simulations[0].state_machines = {'test_sm': self.__sm_mock}
        simulations.append(
            Simulation(1, 'ExDConf/ExDXMLExample.xml', None, 'local', 'default-owner', state='created', context_id='some_uuid'))
        simulations[1].state_machines = {'test_sm': self.__sm_mock}

    def test_all_transitions(self):
        """
        This method tests all transitions (initialize, start, pause, stop, reset).
        """
        transitions.initialize_simulation(0)
        self.assertEqual(simulations[0].cle, "set_cle")
        simulations[0].cle = self.__roscleclient_mock
        self.assertEqual(transitions.initialize_state_machines.call_count, 1)
        transitions.start_simulation(0)
        self.assertEqual(self.__roscleclient_mock.start.call_count, 1)
        self.__sm_mock.start_execution.assert_called_once_with()
        transitions.pause_simulation(0)
        self.assertEqual(self.__roscleclient_mock.pause.call_count, 1)
        transitions.reset_simulation(0)
        self.__sm_mock.request_termination.assert_called_once_with()
        self.assertEqual(self.__sm_mock.wait_termination.call_count, 1)
        self.assertEqual(transitions.initialize_state_machines.call_count, 2)
        self.assertEqual(self.__roscleclient_mock.reset.call_count, 1)
        transitions.stop_simulation(0)
        self.assertEqual(self.__sm_mock.request_termination.call_count, 2)
        self.assertEqual(self.__sm_mock.wait_termination.call_count, 2)

    @mock.patch('hbp_nrp_backend.simulation_control.transitions.shutil.rmtree')
    @mock.patch('hbp_nrp_backend.simulation_control.transitions.initialize_experiment')
    @mock.patch('hbp_nrp_backend.simulation_control.transitions.UserAuthentication')
    def test_simulation_with_context_id_transitions(
        self,
        mock_UserAuthentication,
        mock_initialize_experiment,
        mock_shutil_rmtree
    ):
        tmp_folder_path = '/tmp/tmp_folder'
        exp_path = tmp_folder_path + '/some_exp'
        env_path = tmp_folder_path + '/some_env'
        self.mock_collabClient_instance.clone_experiment_template_from_collab_context.return_value = \
            {'experiment_conf': exp_path , 'environment_conf': env_path}
        transitions.initialize_simulation(1)
        mock_initialize_experiment.assert_called_with(exp_path, env_path, "__generated_experiment_1.py", 1)
        mock_shutil_rmtree.assert_called_with(tmp_folder_path)
        self.assertEqual(mock_shutil_rmtree.call_count, 1)
        simulations[1].cle = self.__roscleclient_mock
        self.assertEqual(transitions.initialize_state_machines.call_count, 1)
        transitions.start_simulation(1)
        self.assertEqual(self.__roscleclient_mock.start.call_count, 1)
        self.__sm_mock.start_execution.assert_called_once_with()
        transitions.pause_simulation(1)
        self.assertEqual(self.__roscleclient_mock.pause.call_count, 1)
        transitions.reset_simulation(1)
        self.__sm_mock.request_termination.assert_called_once_with()
        self.assertEqual(self.__sm_mock.wait_termination.call_count, 1)
        self.assertEqual(transitions.initialize_state_machines.call_count, 2)
        self.assertEqual(self.__roscleclient_mock.reset.call_count, 1)
        transitions.stop_simulation(1)
        self.assertEqual(self.__sm_mock.request_termination.call_count, 2)
        self.assertEqual(self.__sm_mock.wait_termination.call_count, 2)

    @mock.patch('hbp_nrp_backend.simulation_control.transitions.generate_bibi')
    @mock.patch('hbp_nrp_backend.simulation_control.transitions.initialize_experiment')
    def test_initialize_simulation_model_path(self, mock_initialize_experiment, mock_generate_bibi):
        transitions.initialize_simulation(0)
        mock_initialize_experiment.side_effect = rospy.ROSException()
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, 0)
        mock_initialize_experiment.side_effect = None
        mock_generate_bibi.side_effect = IOError()
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, 0)
        mock_generate_bibi.side_effect = None

        transitions.initialize_simulation(0)
        self.assertEquals(mock_generate_bibi.call_count, 4)
        self.assertEquals(mock_initialize_experiment.call_count, 3)


    def test_initialize_ros_exception(self):
        """
        This method simulates a ROSException during simulation initialization
        """

        oldie = transitions.initialize_experiment
        transitions.initialize_experiment = lambda x, y, z, w: (_ for _ in ()).throw(ROSException)
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, 0)
        transitions.initialize_experiment = oldie

    def test_initialize_io_error(self):
        """
        This method induces an IOError during simulation initialization
        """

        oldie = transitions.initialize_experiment
        transitions.initialize_experiment = lambda x, y, z, w: (_ for _ in ()).throw(IOError)
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, 0)
        transitions.initialize_experiment = oldie

if __name__ == '__main__':
    unittest.main()
