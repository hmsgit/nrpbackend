"""
Unit tests for the error handlers
"""

__author__ = 'Georg Hinkel'

import json
import unittest
import hbp_nrp_backend.simulation_control.tests.unit_tests as utc
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server import NRPServicesGeneralException, app
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException


class TestErrorHandlers(RestTest):

    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', None, 'default-owner', 'local', 'view', state='initialized'))
        utc.use_unit_test_transitions()

    def tearDown(self):
        utc.use_production_transitions()

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
        simulations.append(Simulation(0, 'experiment1', None, 'default-owner', 'local', 'view', state='started'))
        # try to restart an already started experiment: state invalid started->created
        response = self.client.put('/simulation/0/state', data='{"state": "created"}')
        self.assertEqual(response.status_code, 400)
        response_object = json.loads(response.data)
        self.assertEqual(u"Invalid transition (started->created)", response_object['message'])
        self.assertEqual(u"Transition error", response_object['type'])


if __name__ == '__main__':
    unittest.main()