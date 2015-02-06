"""
Unit tests for the simulation setup
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations
import unittest
import json


class TestSimulationService(unittest.TestCase):

    def test_simulation_service_post(self):
        client = app.test_client()
        n = len(simulations)

        response = client.post('/simulation', data='{"experimentID":"MyExample.xml"}')

        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.headers['Location'], 'http://localhost/simulation/' + str(n))
        expectedResponseData = {'state': "created", 'simulationID': n, 'experimentID': "MyExample.xml"}
        erd = json.dumps(expectedResponseData)
        self.assertEqual(response.data, erd)
        self.assertEqual(len(simulations), n + 1)
        simulation = simulations[n]
        self.assertEqual(simulation.experiment_id, 'MyExample.xml')

    def test_simulation_service_get(self):
        client = app.test_client()
        n = len(simulations)
        number_of_new_simulations = 4

        for i in range(number_of_new_simulations):
            param = '{"experimentID":"MyExample' + str(n + i) + '.xml"}'
            client.post('/simulation', data=param)
        response = client.get('/simulation')

        self.assertEqual(response.status_code, 200)
        m = n + number_of_new_simulations
        self.assertEqual(len(simulations), m)
        simulation = simulations[m - 1]
        experimentID = 'MyExample' + str(m - 1) + '.xml'
        self.assertEqual(simulation.experiment_id, experimentID)

    def test_simulation_service_no_experiment_id(self):
        client = app.test_client()
        n = len(simulations)

        response = client.post('/simulation', data='{"experimentIDx":"MyExample.xml"}')

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), n)

    def test_simulation_service_wrong_method(self):
        client = app.test_client()
        n = len(simulations)

        response = client.put('/simulation', data='{"experimentID":"MyExample.xml"}')

        self.assertEqual(response.status_code, 405)
        self.assertEqual(len(simulations), n)

if __name__ == '__main__':
    unittest.main()
