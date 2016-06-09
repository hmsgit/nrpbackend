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
        roscleclient_patch = mock.patch('hbp_nrp_backend.cle_interface.ROSCLEClient.ROSCLEClient')
        self.__roscleclient_mock = roscleclient_patch.start()
        self.addCleanup(roscleclient_patch.stop)

        sm_patch = mock.patch("hbp_nrp_backend.simulation_control.Simulation.state_machine_manager")
        self.__sm_man = sm_patch.start()
        self.addCleanup(sm_patch.stop)

        self.__cle = mock.Mock()

        self.__experiment_mock = mock.Mock()
        self.__experiment_mock.timeout = 600
        load_experiment_patch = mock.patch('hbp_nrp_backend.simulation_control.transitions.load_experiment', return_value=self.__experiment_mock)
        load_experiment_patch.start()
        self.addCleanup(load_experiment_patch.stop)

        initialize_experiment_patch = mock.patch('hbp_nrp_backend.simulation_control.transitions.initialize_experiment', return_value=self.__cle)
        initialize_experiment_patch.start()
        self.addCleanup(initialize_experiment_patch.stop)

        generate_experiment_control_patch = mock.patch('hbp_nrp_backend.simulation_control.transitions.generate_experiment_control',
                                                       return_value={'test_sm': 'test_sm.py'})
        generate_experiment_control_patch.start()
        self.addCleanup(generate_experiment_control_patch.stop)

        patch_CollabClient = mock.patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

        self.sim = Simulation(0, 'ExDConf/ExDXMLExample.xml', None, 'default-owner', 'local', state='created')
        self.sim2 = Simulation(1, 'ExDConf/ExDXMLExample.xml', None, 'default-owner', 'local', state='created', context_id='some_uuid')

    def test_all_transitions(self):
        """
        This method tests all transitions (initialize, start, pause, stop, reset).
        """
        transitions.initialize_simulation(self.sim)
        self.assertEqual(self.sim.cle, self.__cle)
        self.sim.cle = self.__roscleclient_mock
        self.assertEqual(self.__sm_man.add_all.call_count, 1)
        self.assertEqual(self.__sm_man.initialize_all.call_count, 1)
        transitions.start_simulation(self.sim)
        self.assertEqual(self.__roscleclient_mock.start.call_count, 1)
        self.__sm_man.start_all.assert_called_once_with()
        transitions.pause_simulation(self.sim)
        self.assertEqual(self.__roscleclient_mock.pause.call_count, 1)
        transitions.reset_simulation(self.sim)
        self.__sm_man.terminate_all.assert_called_once_with()
        self.assertEqual(self.__sm_man.initialize_all.call_count, 1)
        self.assertEqual(self.__roscleclient_mock.reset.call_count, 1)
        transitions.stop_simulation(self.sim)
        self.assertEqual(self.__sm_man.terminate_all.call_count, 2)

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
            {'experiment_conf': 'some_exp', 'environment_conf': 'some_env'}
        transitions.initialize_simulation(self.sim2)
        mock_initialize_experiment.assert_called_with(self.__experiment_mock, "some_exp",
                                                      "some_env", 1, 'local'),
        self.sim2.cle = self.__roscleclient_mock
        self.assertEqual(self.__sm_man.initialize_all.call_count, 1)
        transitions.start_simulation(self.sim2)
        self.assertEqual(self.__roscleclient_mock.start.call_count, 1)
        self.__sm_man.start_all.assert_called_once_with()
        transitions.pause_simulation(self.sim2)
        self.assertEqual(self.__roscleclient_mock.pause.call_count, 1)
        transitions.reset_simulation(self.sim2)
        self.__sm_man.terminate_all.assert_called_once_with()
        self.assertEqual(self.__roscleclient_mock.reset.call_count, 1)
        transitions.stop_simulation(self.sim2)
        self.assertEqual(self.__sm_man.terminate_all.call_count, 2)

    @mock.patch('hbp_nrp_backend.simulation_control.transitions.initialize_experiment')
    def test_initialize_simulation_model_path(self, mock_initialize_experiment):
        transitions.initialize_simulation(self.sim)
        mock_initialize_experiment.side_effect = rospy.ROSException()
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, self.sim)
        mock_initialize_experiment.side_effect = None

        mock_initialize_experiment.side_effect = IOError()
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, self.sim)
        mock_initialize_experiment.side_effect = None

        transitions.initialize_simulation(self.sim)
        self.assertEquals(mock_initialize_experiment.call_count, 4)

    @mock.patch('hbp_nrp_backend.simulation_control.transitions.initialize_experiment', side_effect=ROSException)
    def test_initialize_ros_exception(self, initialize_mock):
        """
        This method simulates a ROSException during simulation initialization
        """
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, self.sim)

    @mock.patch('hbp_nrp_backend.simulation_control.transitions.initialize_experiment', side_effect=IOError)
    def test_initialize_io_error(self, initialize_mock):
        """
        This method induces an IOError during simulation initialization
        """
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, self.sim)

if __name__ == '__main__':
    unittest.main()
