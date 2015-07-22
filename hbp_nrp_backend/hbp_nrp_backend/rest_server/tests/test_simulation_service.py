"""
Unit tests for the simulation setup
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations
from mock import patch, MagicMock
import unittest
import json
import datetime


class TestSimulationService(unittest.TestCase):
    def setUp(self):
        self.now = datetime.datetime.now()
        # Ensure that the patcher is cleaned up correctly even in exceptional cases
        del simulations[:]
        self.client = app.test_client()

    @patch('hbp_nrp_backend.simulation_control.__Simulation.datetime')
    def test_simulation_service_post(self, mocked_date_time):
        mocked_date_time.datetime = MagicMock()
        mocked_date_time.datetime.now = MagicMock(return_value=self.now)

        response = self.client.post('/simulation', data=json.dumps({
            "experimentID": "MyExample.xml",
            "gzserverHost": "local"
        }))

        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.headers['Location'], 'http://localhost/simulation/0')
        expected_response_data = {
            'owner': "default-owner",
            'state': "created",
            'creationDate': self.now.isoformat(),
            'simulationID': 0,
            'experimentID': "MyExample.xml",
            'gzserverHost': 'local',
            'left_screen_color': 'Gazebo/Blue',
            'right_screen_color': 'Gazebo/Blue'
        }
        erd = json.dumps(expected_response_data)
        self.assertEqual(response.data.strip(), erd)
        self.assertEqual(len(simulations), 1)
        simulation = simulations[0]
        self.assertEqual(simulation.experiment_id, 'MyExample.xml')

    def test_simulation_service_wrong_gzserver_host(self):
        wrong_server = "wrong_server"
        response = self.client.post('/simulation', data=json.dumps({
            "experimentID": "MyExample.xml",
            "gzserverHost": wrong_server
        }))
        self.assertEqual(response.status_code, 401)

    def test_simulation_service_get(self):
        param = json.dumps({'experimentID': 'MyExample.xml', 'gzserverHost': 'local'})
        response = self.client.post('/simulation', data=param)
        self.assertEqual(response.status_code, 201)

        response = self.client.get('/simulation')

        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(simulations), 1)
        simulation = simulations[0]
        self.assertEqual(simulation.experiment_id, 'MyExample.xml')
        self.assertEqual(simulation.gzserver_host, 'local')

    def test_simulation_service_no_experiment_id(self):
        rqdata = {
            "experimentIDx": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_wrong_gzserver_host(self):
        rqdata = {
            "experimentID": "MyExample.xml",
            "gzserverHost": "locano"
        }
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 401)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_another_sim_running(self):
        rqdata = {
            "experimentID": "MyExample.xml",
            "gzserverHost": "lugano"
        }
        s = MagicMock('hbp_nrp_backend.simulation_control.Simulation')()
        s.state = 'started'
        self.client.post('/simulation', data=json.dumps(rqdata))
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 402)

    def test_simulation_service_wrong_method(self):
        rqdata = {
            "experimentID": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = self.client.put('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 405)
        self.assertEqual(len(simulations), 0)


if __name__ == '__main__':
    unittest.main()
