"""
Unit tests for the simulation setup
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations
from mock import patch, Mock, MagicMock
import unittest
import json
import datetime


class TestSimulationService(unittest.TestCase):

    def setUp(self):
        self.now = datetime.datetime.now()

    @patch('hbp_nrp_backend.simulation_control.__Simulation.datetime')
    def test_simulation_service_post(self, mocked_date_time):
        client = app.test_client()
        mocked_date_time.datetime = MagicMock()
        mocked_date_time.datetime.now = MagicMock(return_value=self.now)
        n = len(simulations)

        response = client.post('/simulation', data=json.dumps({
                                                               "experimentID": "MyExample.xml",
                                                               "gzserverHost": "local"
                                                              }))

        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.headers['Location'], 'http://localhost/simulation/' + str(n))
        expected_response_data = {
            'owner': "default-owner",
            'state': "created",
            'creationDate': self.now.isoformat(),
            'simulationID': n,
            'experimentID': "MyExample.xml",
            'gzserverHost': 'local'
        }
        erd = json.dumps(expected_response_data)
        self.assertEqual(response.data, erd)
        self.assertEqual(len(simulations), n + 1)
        simulation = simulations[n]
        self.assertEqual(simulation.experiment_id, 'MyExample.xml')

    def test_simulation_service_get(self):
        client = app.test_client()
        n = len(simulations)
        number_of_new_simulations = 4

        for i in range(number_of_new_simulations):
            ex_id = 'MyExample%d.xml' % (n + i, )
            param = json.dumps({'experimentID': ex_id, 'gzserverHost': 'local'})
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

        rqdata = {
            "experimentIDx": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), n)

    def test_simulation_service_wrong_method(self):
        client = app.test_client()
        n = len(simulations)

        rqdata = {
            "experimentID": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = client.put('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 405)
        self.assertEqual(len(simulations), n)

if __name__ == '__main__':
    unittest.main()
