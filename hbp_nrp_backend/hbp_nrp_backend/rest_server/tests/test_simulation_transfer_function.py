"""
Unit tests for the service that patches transfer function sources
"""

__author__ = 'DanielPeppicelli, LucGuyot'

import hbp_nrp_backend
from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, NRPServicesTransferFunctionException
from hbp_nrp_backend.simulation_control import simulations, Simulation
from mock import patch, MagicMock
import unittest
import json

class TestSimulationTransferFunction(unittest.TestCase):

    def setUp(self):
        self.client = app.test_client()

        del simulations[:]
        simulations.append(Simulation(0, 'experiment_0', 'default-owner', 'local', 'created'))
        simulations.append(Simulation(1, 'experiment_1', 'untrusted-owner', 'local', 'created'))

    def test_simulation_transfer_function_put(self):
        sim = simulations[0]
        sim.cle = MagicMock()
        sim.cle.set_simulation_transfer_function = MagicMock(return_value=True)
        client = app.test_client()
        response = client.put('/simulation/0/transferfunctions/incredible_tf_12')
        self.assertEqual(sim.cle.set_simulation_transfer_function.call_count, 1)
        self.assertEqual(response.status_code, 200)

        sim.cle.set_simulation_transfer_function = MagicMock(return_value=False)
        response = client.put('/simulation/0/transferfunctions/stunning_tf_34')
        self.assertRaises(NRPServicesTransferFunctionException)
        self.assertEqual(response.status_code, 400)

        sim = simulations[1]
        response = client.put('/simulation/1/transferfunctions/amazing_tf_35')
        self.assertRaises(NRPServicesClientErrorException)
        self.assertEqual(response.status_code, 401)

if __name__ == '__main__':
    unittest.main()
