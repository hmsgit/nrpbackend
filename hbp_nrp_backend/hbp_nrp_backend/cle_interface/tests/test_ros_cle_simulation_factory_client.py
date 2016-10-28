"""
Unit tests for the ROSCLESimulationFactoryClient
"""
from mock import patch, Mock
from hbp_nrp_backend.cle_interface.ROSCLESimulationFactoryClient import ROSCLESimulationFactoryClient

__author__ = 'Alessandro Ambrosano, Georg'

import unittest

class TestROSCLESimulationFactoryClient(unittest.TestCase):

    @patch('hbp_nrp_backend.cle_interface.ROSCLESimulationFactoryClient.rospy.ServiceProxy')
    def test_create_new_simulation(self, mock_service_proxy):
        roscle = ROSCLESimulationFactoryClient()
        service_proxy = mock_service_proxy()
        self.assertTrue(service_proxy.wait_for_service.called)
        self.assertIsNone(roscle.create_new_simulation(1, 2, 3, 4, 5, 6))
        service_proxy.assert_once_called_with(1, 2, 3, 4, 5, 6)
        service_proxy.side_effect = Exception
        self.assertRaises(Exception, roscle.create_new_simulation, 1, 2, 3, 4, 5, 6)


if __name__ == '__main__':
    unittest.main()
