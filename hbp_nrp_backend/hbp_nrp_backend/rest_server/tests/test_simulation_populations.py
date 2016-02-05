"""
Unit tests for the simulation neurons rest call
"""

__author__ = 'Bernd Eckstein'

import unittest
import mock
import rospy
import json
from hbp_nrp_backend.rest_server.tests import RestTest

ros_service_object = mock.Mock()
rospy.wait_for_service = mock.Mock(return_value=ros_service_object)
rospy.ServiceProxy = mock.Mock(return_value=ros_service_object)

from hbp_nrp_backend.rest_server import init
from hbp_nrp_backend.simulation_control import simulations, Simulation

neurons = {'populations': [
              {
               'name': "Test",
               'neuron_model': "NeuronModel",
               'parameters': [
                  {'parameterName': "string",
                   'value': 0.0}],
               'gids': [0]
              }
            ]
           }

class TestSimulationService(RestTest):

    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', None, 'default-owner', 'created'))
        simulations[0].cle = mock.MagicMock()
        simulations[0].cle.get_populations = mock.MagicMock(return_value=neurons)

    def test_get_neurons_sim_ok(self):
        response = self.client.get('/simulation/0/populations')
        self.assertEqual(200, response.status_code)
        # deserialilze
        neurons = json.loads(response.data)
        self.assertEqual("NeuronModel", neurons['populations'][0]['neuron_model'])

    def test_get_neurons_sim_not_found(self):
        response = self.client.get('/simulation/1/populations')
        self.assertEqual(404, response.status_code)

if __name__ == '__main__':
    unittest.main()
