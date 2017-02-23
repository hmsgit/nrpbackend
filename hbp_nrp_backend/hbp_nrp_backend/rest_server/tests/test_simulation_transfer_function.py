"""
Unit tests for the service that patches transfer function sources
"""

__author__ = 'DanielPeppicelli, LucGuyot'

import unittest
from mock import MagicMock
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, NRPServicesTransferFunctionException
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server.tests import RestTest


class TestSimulationTransferFunction(RestTest):

    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment_0', None, 'default-owner', 'local', 'created'))
        simulations.append(Simulation(1, 'experiment_1', None, 'untrusted-owner', 'local', 'created'))
        self.sim = simulations[0]
        self.sim.cle = MagicMock()
        self.sim.cle.set_simulation_transfer_function = MagicMock(return_value="")

    def test_simulation_transfer_function_put(self):
        response = self.client.put('/simulation/0/transfer-functions/incredible_tf_12')
        self.assertEqual(self.sim.cle.set_simulation_transfer_function.call_count, 1)
        self.assertEqual(response.status_code, 200)

        self.sim.cle.set_simulation_transfer_function = MagicMock(return_value="error")
        response = self.client.put('/simulation/0/transfer-functions/stunning_tf_34')
        self.assertRaises(NRPServicesTransferFunctionException)
        self.assertEqual(response.status_code, 400)

        sim = simulations[1]
        response = self.client.put('/simulation/1/transfer-functions/amazing_tf_35')
        self.assertRaises(NRPServicesClientErrorException)
        self.assertEqual(response.status_code, 403)

    def test_simulation_transfer_function_delete(self):
        self.sim.cle.delete_simulation_transfer_function = MagicMock(return_value=True)
        response = self.client.delete('/simulation/0/transfer-functions/incredible_tf_12')
        self.assertEqual(self.sim.cle.delete_simulation_transfer_function.call_count, 1)
        self.assertEqual(response.status_code, 200)

        self.sim.cle.delete_simulation_transfer_function = MagicMock(return_value=False)
        response = self.client.delete('/simulation/0/transfer-functions/stunning_tf_34')
        self.assertRaises(NRPServicesTransferFunctionException)
        self.assertEqual(response.status_code, 400)

        sim = simulations[1]
        response = self.client.delete('/simulation/1/transfer-functions/amazing_tf_35')
        self.assertRaises(NRPServicesClientErrorException)
        self.assertEqual(response.status_code, 403)

if __name__ == '__main__':
    unittest.main()
