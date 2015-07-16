"""
Unit tests for the simulation control
"""

__author__ = 'GeorgHinkel'

import unittest

import hbp_nrp_backend.simulation_control.tests.unit_tests as utc
from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException


class TestSimulationConfig(unittest.TestCase):
    def setUp(self):
        self.client = app.test_client()

        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', 'default-owner', 'local', 'created'))
        simulations.append(Simulation(1, 'experiment2', 'default-owner', 'local', 'initialized'))
        simulations.append(Simulation(2, 'experiment3', 'default-owner', 'local', 'started'))
        simulations.append(Simulation(3, 'experiment3', 'default-owner', 'local', 'started'))
        simulations.append(Simulation(4, 'experiment3', 'default-owner', 'local', 'started'))
        simulations.append(Simulation(5, 'experiment4', 'default-owner', 'local', 'paused'))
        simulations.append(Simulation(6, 'experiment4', 'default-owner', 'local', 'paused'))
        simulations.append(Simulation(7, 'experiment4', 'default-owner', 'local', 'paused'))
        simulations.append(Simulation(8, 'experiment5', 'default-owner', 'local', 'stopped'))

        utc.use_unit_test_transitions()

    def test_get_simulation(self):
        response = self.client.get('/simulation/0')
        self.assertTrue('"experimentID": "experiment1"' in response.data)
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/1')
        self.assertTrue('"experimentID": "experiment2"' in response.data)
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/2')
        self.assertTrue('"experimentID": "experiment3"' in response.data)
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/5')
        self.assertTrue('"experimentID": "experiment4"' in response.data)
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/8')
        self.assertTrue('"experimentID": "experiment5"' in response.data)
        self.assertEqual(response.status_code, 200)

    def test_get_state(self):
        response = self.client.get('/simulation/0/state')
        self.assertEqual('{"state": "created"}', response.data.strip())
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/1/state')
        self.assertEqual('{"state": "initialized"}', response.data.strip())
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/2/state')
        self.assertEqual('{"state": "started"}', response.data.strip())
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/5/state')
        self.assertEqual('{"state": "paused"}', response.data.strip())
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/8/state')
        self.assertEqual('{"state": "stopped"}', response.data.strip())
        self.assertEqual(response.status_code, 200)

    def test_created_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/0/state', data='{"state": "created"}')
        self.assertEqual(response.status_code, 400)
        self.assertTrue(utc.last_sim_id is None)

        response = self.client.put('/simulation/0/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 400)
        self.assertTrue(utc.last_sim_id is None)

        response = self.client.put('/simulation/0/state', data='{"state": "paused"}')
        self.assertEqual(response.status_code, 400)
        self.assertTrue(utc.last_sim_id is None)

        response = self.client.put('/simulation/0/state', data='{"state": "stopped"}')
        self.assertEqual(response.status_code, 400)
        self.assertTrue(utc.last_sim_id is None)

        response = self.client.put('/simulation/20/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 404)
        self.assertTrue(utc.last_sim_id is None)

        response = self.client.put('/simulation/0/state', data='{"state": "initialized"}',
                                   headers={'X-User-Name': 'Test'})
        self.assertEqual(response.status_code, 401)
        self.assertTrue(utc.last_sim_id is None)

        response = self.client.put('/simulation/0/state', data='{"state": "initialized"}')
        self.assertEqual(response.status_code, 200)
        self.assertEqual('{"state": "initialized"}', response.data.strip())
        self.assertEqual(utc.last_sim_id, 0)
        self.assertEqual(utc.last_transition, "initialize")

    def test_initialized_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/1/state', data='{"state": "initialized"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/1/state', data='{"state": "created"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/1/state', data='{"state": "paused"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/1/state', data='{"state": "started"}',
                                   headers={'X-User-Name': 'Test'})
        self.assertEqual(response.status_code, 401)
        self.assertTrue(utc.last_sim_id is None)

        response = self.client.put('/simulation/1/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 200)
        self.assertEqual('{"state": "started"}', response.data.strip())
        self.assertEqual(utc.last_sim_id, 1)
        self.assertEqual(utc.last_transition, "start")

        response = self.client.put('/simulation/1/state', data='{"state": "stopped"}')
        self.assertEqual(response.status_code, 200)
        self.assertEqual('{"state": "stopped"}', response.data.strip())
        self.assertIs(utc.last_sim_id, 1)

    def test_started_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/2/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/2/state', data='{"state": "created"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/2/state', data='{"state": "paused"}')
        self.assertEqual(response.status_code, 200)
        self.assertEqual('{"state": "paused"}', response.data.strip())
        self.assertEqual(utc.last_sim_id, 2)
        self.assertEqual(utc.last_transition, "pause")

        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/3/state', data='{"state": "stopped"}',
                                   headers={'X-User-Name': 'Test'})
        self.assertEqual(response.status_code, 401)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/3/state', data='{"state": "stopped"}')
        self.assertEqual(response.status_code, 200)
        self.assertEqual('{"state": "stopped"}', response.data.strip())
        self.assertEqual(utc.last_sim_id, 3)
        self.assertEqual(utc.last_transition, "stop")

        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/4/state', data='{"state": "initialized"}')
        self.assertEqual(response.status_code, 200)
        self.assertEqual('{"state": "initialized"}', response.data.strip())
        self.assertEqual(utc.last_sim_id, 4)
        self.assertEqual(utc.last_transition, "reset")

    def test_paused_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/5/state', data='{"state": "paused"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/5/state', data='{"state": "created"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/5/state', data='{"state": "started"}',
                                   headers={'X-User-Name': 'Test'})
        self.assertEqual(response.status_code, 401)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/5/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 200)
        self.assertEqual('{"state": "started"}', response.data.strip())
        self.assertEqual(utc.last_sim_id, 5)
        self.assertEqual(utc.last_transition, "start")

        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/6/state', data='{"state": "stopped"}')
        self.assertEqual(response.status_code, 200)
        self.assertIn('{"state": "stopped"}', response.data.strip())
        self.assertEqual(utc.last_sim_id, 6)
        self.assertEqual(utc.last_transition, "stop")

        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/7/state', data='{"state": "initialized"}')
        self.assertEqual(response.status_code, 200)
        self.assertEqual('{"state": "initialized"}', response.data.strip())
        self.assertEqual(utc.last_sim_id, 7)
        self.assertEqual(utc.last_transition, "reset")

    def test_stopped_transitions(self):
        utc.last_sim_id = None
        utc.last_transition = None

        response = self.client.put('/simulation/8/state', data='{"state": "stopped"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/8/state', data='{"state": "created"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/8/state', data='{"state": "initialized"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/8/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/8/state', data='{"state": "paused"}')
        self.assertEqual(response.status_code, 400)
        self.assertIs(utc.last_sim_id, None)

        response = self.client.put('/simulation/8/state', data='{"state": "paused"}',
                                   headers={'X-User-Name': 'Test'})
        self.assertEqual(response.status_code, 401)
        self.assertIs(utc.last_sim_id, None)

    def test_properties(self):

        class Mock1(object):
            def get_simulation_state(self):
                return 'stopped_mock'

        class Mock2(object):
            def get_simulation_state(self):
                raise ROSCLEClientException('exception')

        simulations[0].cle = Mock1()
        self.assertEqual(simulations[0].state, 'stopped_mock')

        simulations[0].cle = Mock2()
        self.assertEqual(simulations[0].state, 'stopped')
