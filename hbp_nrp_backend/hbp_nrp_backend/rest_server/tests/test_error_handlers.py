"""
Unit tests for the error handlers
"""

__author__ = 'GeorgHinkel'

import unittest
import json
import hbp_nrp_backend.simulation_control.tests.unit_tests as utc
from hbp_nrp_backend.rest_server import app, NRPServicesGeneralException
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException


class TestErrorHandlers(unittest.TestCase):
    def setUp(self):
        self.client = app.test_client()
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', 'default-owner', 'local', 'view', 'initialized'))
        utc.use_unit_test_transitions()

    def test_general_500_error(self):
        utc.start_will_raise_exception(Exception("I am a general Exception"))
        response = self.client.put('/simulation/0/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 500)
        response_object = json.loads(response.data)
        self.assertEqual(u"Internal server error: I am a general Exception",
                         response_object['message'])
        self.assertEqual(u"General error", response_object['type'])

    def test_ros_client_exception(self):
        utc.start_will_raise_exception(ROSCLEClientException("I am a ROSCLEClientException"))
        response = self.client.put('/simulation/0/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 500)
        response_object = json.loads(response.data)
        self.assertEqual(
            u"Error while communicating with the CLE (I am a ROSCLEClientException).",
            response_object['message'])
        self.assertEqual(u"CLE error", response_object['type'])

    def test_nrp_services_general_exception(self):
        utc.start_will_raise_exception(
            NRPServicesGeneralException("I am a NRPServicesGeneralException message",
                                        "I am a NRPServicesGeneralException type"))
        response = self.client.put('/simulation/0/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 500)
        response_object = json.loads(response.data)
        self.assertEqual(u"I am a NRPServicesGeneralException message", response_object['message'])
        self.assertEqual(u"I am a NRPServicesGeneralException type", response_object['type'])

    def test_nrp_services_state_exception(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', 'default-owner', 'local', 'view', 'started'))
        # try to start an already started experiment: state invalid started->started
        response = self.client.put('/simulation/0/state', data='{"state": "started"}')
        self.assertEqual(response.status_code, 400)
        response_object = json.loads(response.data)
        self.assertEqual(u"Invalid transition (started->started)", response_object['message'])
        self.assertEqual(u"Transition error", response_object['type'])
