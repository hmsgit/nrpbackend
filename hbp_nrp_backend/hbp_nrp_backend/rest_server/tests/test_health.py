"""
Unit tests for the service that patches transfer function sources
"""

__author__ = 'DanielPeppicelli, LucGuyot'

import hbp_nrp_backend
from datetime import datetime, timedelta
from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, NRPServicesTransferFunctionException
from hbp_nrp_backend.simulation_control import simulations, Simulation
import unittest
import json

class TestHealth(unittest.TestCase):

    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment_0', 'default-owner', 'local', 'created'))
        simulations.append(Simulation(1, 'experiment_1', 'untrusted-owner', 'local', 'created'))
        simulations.append(Simulation(2, 'experiment_2', 'untrusted-owner', 'local', 'created'))
        self.client = app.test_client()

    def testErrors(self):
        response = self.client.get('/health/errors')
        self.assertEqual(response.status_code, 200)
        self.assertIn("OK", response.data)
        simulations[0]._Simulation__errors = 1
        response = self.client.get('/health/errors')
        self.assertEqual(response.status_code, 200)
        self.assertIn("WARNING", response.data)
        simulations[0]._Simulation__errors = 2
        response = self.client.get('/health/errors')
        self.assertEqual(response.status_code, 200)
        self.assertIn("CRITICAL", response.data)

    def testErrorsLast24h(self):
        simulations[0]._Simulation__errors = 1
        simulations[0]._Simulation__creation_datetime = datetime.utcnow() - timedelta(days=1, seconds=2)
        response = self.client.get('/health/errors-last-24h')
        self.assertEqual(response.status_code, 200)
        self.assertIn("OK", response.data)
        simulations[1]._Simulation__errors = 1
        simulations[1]._Simulation__creation_datetime = datetime.utcnow() - timedelta(hours=23)
        simulations[2]._Simulation__creation_datetime = datetime.utcnow() - timedelta(hours=23)
        response = self.client.get('/health/errors-last-24h')
        self.assertEqual(response.status_code, 200)
        self.assertIn("WARNING", response.data)