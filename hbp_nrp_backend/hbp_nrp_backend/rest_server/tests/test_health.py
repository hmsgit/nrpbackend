"""
Unit tests for the service that patches transfer function sources
"""

__author__ = 'Daniel Peppicelli, Luc Guyot'

import unittest
from hbp_nrp_backend.rest_server.tests import RestTest
from datetime import datetime, timedelta
from hbp_nrp_backend.simulation_control import simulations, Simulation
from pytz import timezone


tz = timezone('Europe/Zurich')


class SimulationMock(object):
    errors = Simulation.errors

    def __init__(self):
        self.creation_datetime = datetime.now(tz)


class TestHealth(RestTest):

    def _create_simulation(self, sim_id, experiment_conf, owner, gzserver_host, state):
        m = SimulationMock()
        m.sim_id = sim_id
        m.experiment_conf = experiment_conf
        m.owner = owner
        m.gzserver_host = gzserver_host
        m.state = state
        return m

    def setUp(self):
        del simulations[:]
        simulations.append(self._create_simulation(0, 'experiment_0', 'default-owner', 'local', 'created'))
        simulations.append(self._create_simulation(1, 'experiment_1', 'untrusted-owner', 'local', 'created'))
        simulations.append(self._create_simulation(2, 'experiment_2', 'untrusted-owner', 'local', 'created'))

    def testErrors(self):
        response = self.client.get('/health/errors')
        self.assertEqual(response.status_code, 200)
        self.assertIn("OK", response.data)
        simulations[0].state = 'halted'
        response = self.client.get('/health/errors')
        self.assertEqual(response.status_code, 200)
        self.assertIn("WARNING", response.data)
        simulations[1].state = 'failed'
        response = self.client.get('/health/errors')
        self.assertEqual(response.status_code, 200)
        self.assertIn("CRITICAL", response.data)

    def testErrorsLast24h(self):
        simulations[0].state = 'halted'
        simulations[0].creation_datetime = datetime.now(
            tz=tz) - timedelta(days=1, seconds=2)
        response = self.client.get('/health/errors-last-24h')
        self.assertEqual(response.status_code, 200)
        self.assertIn("OK", response.data)
        simulations[1].state = 'failed'
        simulations[1].creation_datetime = datetime.now(
            tz=tz) - timedelta(hours=23)
        simulations[2].creation_datetime = datetime.now(
            tz=tz) - timedelta(hours=23)
        response = self.client.get('/health/errors-last-24h')
        self.assertEqual(response.status_code, 200)
        self.assertIn("WARNING", response.data)


if __name__ == '__main__':
    unittest.main()
