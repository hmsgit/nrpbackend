"""
Test file for testing hbp_nrp_backend.simulation_control.transitions
"""

__author__ = 'Alessandro Ambrosano'

import unittest
import mock
import os
from rospy import ROSException
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
from hbp_nrp_backend.simulation_control import simulations, Simulation, transitions


class TestTransition(unittest.TestCase):
    """
    Class for testing hbp_nrp_backend.simulation_control.transitions
    """

    def setUp(self):
        self.__roscleclient_mock = mock.patch('hbp_nrp_cle.cle.ROSCLEClient.ROSCLEClient').start()
        self.__roscleclient_mock.start = mock.Mock()
        self.__roscleclient_mock.pause = mock.Mock()
        self.__roscleclient_mock.stop = mock.Mock()
        self.__roscleclient_mock.reset = mock.Mock()

        transitions.generate_bibi = mock.Mock(return_value=mock.Mock())
        transitions.initialize_experiment = mock.Mock(return_value="set_cle")

        del simulations[:]
        simulations.append(
            Simulation(0, 'virtual_room/virtual_room.sdf', 'local', 'default-owner', 'created'))

    def test_all_transitions(self):
        """
        This method tests all transitions (initialize, start, pause, stop, reset).
        """

        transitions.initialize_simulation(0)
        self.assertEqual(simulations[0].cle, "set_cle")
        simulations[0].cle = self.__roscleclient_mock
        transitions.start_simulation(0)
        self.assertEqual(self.__roscleclient_mock.start.call_count, 1)
        transitions.pause_simulation(0)
        self.assertEqual(self.__roscleclient_mock.pause.call_count, 1)
        transitions.reset_simulation(0)
        self.assertEqual(self.__roscleclient_mock.reset.call_count, 1)
        transitions.stop_simulation(0)
        self.assertEqual(self.__roscleclient_mock.stop.call_count, 1)

    def test_initialize_ros_exception(self):
        """
        This method simulates a ROSException during simulation initialization
        """

        oldie = transitions.initialize_experiment
        transitions.initialize_experiment = lambda x, y: (_ for _ in ()).throw(ROSException)
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, 0)
        transitions.initialize_experiment = oldie

    def test_initialize_io_error(self):
        """
        This method induces an IOError during simulation initialization
        """

        oldie = transitions.initialize_experiment
        transitions.initialize_experiment = lambda x, y: (_ for _ in ()).throw(IOError)
        self.assertRaises(NRPServicesGeneralException, transitions.initialize_simulation, 0)
        transitions.initialize_experiment = oldie

if __name__ == '__main__':
    unittest.main()
