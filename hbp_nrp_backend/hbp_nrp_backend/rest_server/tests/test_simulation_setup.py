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

        response = client.post('/simulation', data='{"experimentID":"MyDumbExample.xml"}')

        assert response.status_code == 201
        self.assertEqual(response.headers['Location'], 'http://localhost/simulation/' + str(n) + '/state')
        assert len(simulations) == n + 1

    def test_simulation_setup_no_experiment_id(self):
        client = app.test_client()

        n = len(simulations)

        response = client.post('/simulation', data='{"experimentIDx":"MyDumbExample.xml"}')

        assert response.status_code == 400
        assert len(simulations) == n

    def test_simulation_setup_wrong_method(self):
        client = app.test_client()

        n = len(simulations)

        response = client.get('/simulation', data='{"experimentIDx":"MyDumbExample.xml"}')

        assert response.status_code == 405
        assert len(simulations) == n

if __name__ == '__main__':
    unittest.main()