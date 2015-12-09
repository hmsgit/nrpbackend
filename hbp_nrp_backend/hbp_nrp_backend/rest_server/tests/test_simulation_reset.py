"""
Unit tests for the __SimulationReset module.
"""

__author__ = 'Alessandro Ambrosano'

import unittest
import mock
import json

from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.simulation_control import simulations, Simulation


class TestSimulationReset(RestTest):

    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', None, 'default-owner', 'created'))
        simulations.append(Simulation(1, 'experiment2', None, 'im-not-the-owner', 'created'))

    def test_get_reset(self):
        simulations[0].cle = mock.MagicMock()

        # Correct request
        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'oldReset': False,
            'robotPose': False,
            'fullReset': False
        }))
        self.assertEqual(200, response.status_code)
        simulations[0].cle.reset.assert_called()

        # Invalid request, too much parameters
        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'oldReset': False,
            'robotPose': False,
            'fullReset': False,
            'randomInvalidParameter': False
        }))
        self.assertEqual(400, response.status_code)

        # Invalid request, missing parameters
        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'oldReset': False,
            'robotPose': False
        }))
        self.assertEqual(400, response.status_code)

        # This simulation doesn't exist
        response = self.client.put('/simulation/2/reset')
        self.assertEqual(404, response.status_code)

        # I'm not the owner of this one
        response = self.client.put('/simulation/1/reset')
        self.assertEqual(401, response.status_code)

        # Now the request is fine, but something goes wrong out of the backend reach
        simulations[0].cle.reset.side_effect = ROSCLEClientException()
        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'oldReset': False,
            'robotPose': False,
            'fullReset': False
        }))
        self.assertEqual(500, response.status_code)


if __name__ == '__main__':
    unittest.main()
