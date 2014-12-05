"""
Unit tests for the simulation control
"""

__author__ = 'GeorgHinkel'

import unittest

import hbp_nrp_backend.simulation_control.unit_tests as utc
from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations, Simulation


class TestSimulationConfig(unittest.TestCase):
    def setUp(self):
        self.client = app.test_client()

        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', 'created'))
        simulations.append(Simulation(1, 'experiment2', 'initialized'))
        simulations.append(Simulation(2, 'experiment3', 'started'))
        simulations.append(Simulation(3, 'experiment3', 'started'))
        simulations.append(Simulation(4, 'experiment3', 'started'))
        simulations.append(Simulation(5, 'experiment4', 'paused'))
        simulations.append(Simulation(6, 'experiment4', 'paused'))
        simulations.append(Simulation(7, 'experiment4', 'paused'))
        simulations.append(Simulation(8, 'experiment5', 'stopped'))

        utc.use_unit_test_transitions()

    def test_get_state(self):
        response = self.client.get('/simulation/0/state')
        assert '"state": "created"' in response.data
        assert '"experimentID": "experiment1"' in response.data
        assert response.status_code == 200

        response = self.client.get('/simulation/1/state')
        assert '"state": "initialized"' in response.data
        assert '"experimentID": "experiment2"' in response.data
        assert response.status_code == 200

	response = self.client.get('/simulation/2/state')
        assert '"state": "started"' in response.data
        assert '"experimentID": "experiment3"' in response.data
        assert response.status_code == 200

        response = self.client.get('/simulation/5/state')
        assert '"state": "paused"' in response.data
        assert '"experimentID": "experiment4"' in response.data
        assert response.status_code == 200

        response = self.client.get('/simulation/8/state')
        assert '"state": "stopped"' in response.data
        assert '"experimentID": "experiment5"' in response.data
        assert response.status_code == 200

    def test_created_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/0/state', data='{"state": "created"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/0/state', data='{"state": "started"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/0/state', data='{"state": "paused"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/0/state', data='{"state": "stopped"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/20/state', data='{"state": "started"}')
        assert response.status_code == 404
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/0/state', data='{"state": "initialized"}')
        assert response.status_code == 200
        assert '"state": "initialized"' in response.data
        assert '"experimentID": "experiment1"' in response.data
        assert utc.last_sim_id == 0
        assert utc.last_transition == "initialize"

    def test_initialized_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/1/state', data='{"state": "initialized"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/1/state', data='{"state": "created"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/1/state', data='{"state": "paused"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/1/state', data='{"state": "stopped"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/1/state', data='{"state": "started"}')
        assert response.status_code == 200
        assert '"state": "started"' in response.data
        assert '"experimentID": "experiment2"' in response.data
        assert utc.last_sim_id == 1
        assert utc.last_transition == "start"

    def test_started_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/2/state', data='{"state": "started"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/2/state', data='{"state": "created"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/2/state', data='{"state": "paused"}')
        assert response.status_code == 200
        assert '"state": "paused"' in response.data
        assert '"experimentID": "experiment3"' in response.data
        assert utc.last_sim_id == 2
        assert utc.last_transition == "pause"

        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/3/state', data='{"state": "stopped"}')
        assert response.status_code == 200
        assert '"state": "stopped"' in response.data
        assert '"experimentID": "experiment3"' in response.data
        assert utc.last_sim_id == 3
        assert utc.last_transition == "stop"

        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/4/state', data='{"state": "initialized"}')
        assert response.status_code == 200
        assert '"state": "initialized"' in response.data
        assert '"experimentID": "experiment3"' in response.data
        assert utc.last_sim_id == 4
        assert utc.last_transition == "reset"

    def test_paused_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/5/state', data='{"state": "paused"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/5/state', data='{"state": "created"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/5/state', data='{"state": "started"}')
        assert response.status_code == 200
        assert '"state": "started"' in response.data
        assert '"experimentID": "experiment4"' in response.data
        assert utc.last_sim_id == 5
        assert utc.last_transition == "start"

        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/6/state', data='{"state": "stopped"}')
        assert response.status_code == 200
        assert '"state": "stopped"' in response.data
        assert '"experimentID": "experiment4"' in response.data
        assert utc.last_sim_id == 6
        assert utc.last_transition == "stop"

        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/7/state', data='{"state": "initialized"}')
        assert response.status_code == 200
        assert '"state": "initialized"' in response.data
        assert '"experimentID": "experiment4"' in response.data
        assert utc.last_sim_id == 7
        assert utc.last_transition == "reset"

    def test_stopped_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/8/state', data='{"state": "stopped"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/8/state', data='{"state": "created"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/8/state', data='{"state": "initialized"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/8/state', data='{"state": "started"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None

        response = self.client.put('/simulation/8/state', data='{"state": "paused"}')
        assert response.status_code == 400
        assert utc.last_sim_id is None
