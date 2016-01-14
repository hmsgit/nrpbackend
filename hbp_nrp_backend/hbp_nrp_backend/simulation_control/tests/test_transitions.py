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
        self.__roscleclient_patch = mock.patch('hbp_nrp_backend.cle_interface.ROSCLEClient.ROSCLEClient')
        self.__roscleclient_mock = self.__roscleclient_patch.start()
        self.__roscleclient_mock.start = mock.Mock()
        self.__roscleclient_mock.pause = mock.Mock()
        self.__roscleclient_mock.stop = mock.Mock()
        self.__roscleclient_mock.reset = mock.Mock()
        self.__sm_patch = mock.patch("hbp_nrp_backend.simulation_control.Simulation.state_machine_manager")
        self.__sm_man = self.__sm_patch.start()

        transitions.initialize_experiment = mock.Mock(return_value="set_cle")
        transitions.generate_experiment_control = mock.Mock(return_value={'test_sm': 'test_sm.py'})

        patch_CollabClient = mock.patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

        self.sim = Simulation(0, 'ExDConf/ExDXMLExample.xml', None, 'default-owner', 'local', state='created')
        self.sim2 = Simulation(1, 'ExDConf/ExDXMLExample.xml', None, 'default-owner', 'local', state='created', context_id='some_uuid')

    def tearDown(self):
        self.__roscleclient_patch.stop()
        self.__sm_patch.stop()

    def test_all_transitions(self):
        """
        This method tests all transitions (initialize, start, pause, stop, reset).
        """
        transitions.initialize_simulation(self.sim)
        self.assertEqual(self.sim.cle, "set_cle")
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
        mock_initialize_experiment.assert_called_with("some_exp", "some_env", 1, \
                                                                 'local'),
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


    def test_initialize_ros_exception(self):
        """
        This method simulates a ROSException during simulation initialization
        """

        oldie = transitions.initialize_experiment
        transitions.initialize_experiment = lambda x, y, z, w: (_ for _ in ()).throw(
                ROSException)
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, self.sim)

        transitions.initialize_experiment = oldie

    def test_initialize_io_error(self):
        """
        This method induces an IOError during simulation initialization
        """

        oldie = transitions.initialize_experiment
        transitions.initialize_experiment = lambda x, y, z, w: (_ for _ in ()).throw(IOError)
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, self.sim)

        transitions.initialize_experiment = oldie

if __name__ == '__main__':
    unittest.main()
