"""
Unit tests for the simulation setup
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations
import unittest


class TestSimulationSetup(unittest.TestCase):

    def test_simulation_setup(self):
        client = app.test_client()
        n = len(simulations)

        response = client.post('/simulation', data='{"experimentID":"MyExample.xml"}')

        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.headers['Location'], 'http://localhost/simulation/' + str(n) + '/state')
        self.assertEqual(len(simulations), n + 1)

    def test_simulation_setup_get(self):
        client = app.test_client()
        n = len(simulations)
        numberOfNewSimulations = 4

        for i in range(numberOfNewSimulations):
            param = '{"experimentID":"MyExample' + str(i) + '.xml"}'
            r = client.post('/simulation', data=param)
        response = client.get('/simulation')

        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(simulations), n + numberOfNewSimulations)

    def test_simulation_setup_no_experiment_id(self):
        client = app.test_client()
        n = len(simulations)

        response = client.post('/simulation', data='{"experimentIDx":"MyExample.xml"}')

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), n)

    def test_simulation_setup_wrong_method(self):
        client = app.test_client()
        n = len(simulations)

        response = client.put('/simulation', data='{"experimentID":"MyExample.xml"}')

        self.assertEqual(response.status_code, 405)
        self.assertEqual(len(simulations), n)

if __name__ == '__main__':
    unittest.main()
