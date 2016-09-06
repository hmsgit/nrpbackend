"""
Unit tests for the simulation setup
"""

__author__ = 'GeorgHinkel'


import unittest
import json
import datetime
from mock import patch, MagicMock
from hbp_nrp_backend.simulation_control import simulations
from hbp_nrp_backend.rest_server.tests import RestTest


class TestSimulationService(RestTest):
    def setUp(self):
        self.now = datetime.datetime.now()
        # Ensure that the patcher is cleaned up correctly even in exceptional cases
        del simulations[:]

    @patch('hbp_nrp_backend.simulation_control.__Simulation.datetime')
    def test_simulation_service_post(self, mocked_date_time):
        mocked_date_time.datetime = MagicMock()
        mocked_date_time.datetime.now = MagicMock(return_value=self.now)

        response = self.client.post('/simulation',
                                    data=json.dumps({"experimentConfiguration": "MyExample.xml",
                                                     "gzserverHost": "local"}))

        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.headers['Location'], 'http://localhost/simulation/0')
        expected_response_data = {
            'owner': "default-owner",
            'state': "created",
            'creationDate': self.now.isoformat(),
            'simulationID': 0,
            'experimentConfiguration': "MyExample.xml",
            'environmentConfiguration': None,
            'gzserverHost': 'local',
            'contextID': None,
            'brainProcesses': 1,
        }
        erd = json.dumps(expected_response_data)
        self.assertEqual(response.data.strip(), erd)
        self.assertEqual(len(simulations), 1)
        simulation = simulations[0]
        self.assertEqual(simulation.experiment_conf, 'MyExample.xml')

    def test_simulation_service_wrong_gzserver_host(self):
        wrong_server = "wrong_server"
        response = self.client.post('/simulation',
                                    data=json.dumps({"experimentConfiguration": "MyExample.xml",
                                                     "gzserverHost": wrong_server}))
        self.assertEqual(response.status_code, 401)

    def test_simulation_service_get(self):
        ctx_id = '0a008f825ed94400110cba4700725e4dff2f55d1'
        param = json.dumps({
            'experimentConfiguration': 'MyExample.xml',
            'gzserverHost': 'local',
            'contextID': ctx_id
        })
        response = self.client.post('/simulation', data=param)
        self.assertEqual(response.status_code, 201)

        response = self.client.get('/simulation')

        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(simulations), 1)
        simulation = simulations[0]
        self.assertEqual(simulation.experiment_conf, 'MyExample.xml')
        self.assertEqual(simulation.gzserver_host, 'local')
        self.assertEqual(simulation.context_id, ctx_id)

    def test_simulation_service_no_experiment_conf(self):
        rqdata = {
            "experimentConfiguration_missing": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        print response
        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_wrong_gzserver_host(self):
        rqdata = {
            "experimentConfiguration": "MyExample.xml",
            "gzserverHost": "luxano"   # Wrong server here
        }
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 401)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_another_sim_running(self):
        rqdata = {
            "experimentConfiguration": "MyExample.xml",
            "gzserverHost": "lugano"
        }
        s = MagicMock('hbp_nrp_backend.simulation_control.Simulation')()
        s.state = 'started'
        self.client.post('/simulation', data=json.dumps(rqdata))
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 409)

    def test_simulation_service_wrong_method(self):
        rqdata = {
            "experimentConfiguration": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = self.client.put('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 405)
        self.assertEqual(len(simulations), 0)


if __name__ == '__main__':
    unittest.main()
