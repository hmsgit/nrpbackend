"""
Unit tests for the simulation configuration
"""

__author__ = 'GeorgHinkel'

import unittest
import mock
import rospy
from hbp_nrp_backend.rest_server.tests import RestTest

ros_service_object = mock.Mock()
rospy.wait_for_service = mock.Mock(return_value=ros_service_object)
rospy.ServiceProxy = mock.Mock(return_value=ros_service_object)

from hbp_nrp_backend.rest_server import init
from hbp_nrp_backend.simulation_control import simulations, Simulation


class TestSimulationService(RestTest):
    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', None, 'default-owner', 'created'))

    def test_put_light(self):
        response = self.client.put('/simulation/0/interaction/light', data='{"name":"foo"}')
        self.assertEqual('"Changed light intensity"', response.data.strip())
        self.assertEqual(200, response.status_code)

if __name__ == '__main__':
    unittest.main()
