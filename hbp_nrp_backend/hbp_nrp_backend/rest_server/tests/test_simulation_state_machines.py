"""
Unit tests for the service that pushes and gets state machines
"""

__author__ = "Bernd Eckstein"

import unittest
import os
import json
from mock import patch, Mock
from flask import Response, Request
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server.__SimulationControl import UserAuthentication
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server import NRPServicesGeneralException, \
    NRPServicesStateMachineException, NRPServicesWrongUserException


SM = """\
import mock
import time
sm=mock.Mock()
sm.execute=time.sleep
"""

PATH = os.path.split(__file__)[0]


class TestSimulationStateMachines(RestTest):

    os.environ['NRP_MODELS_DIRECTORY'] = PATH

    def setUp(self):
        load_data = {
            "experimentConfiguration": "experiments/experiment_data/test_1.exc",
            "gzserverHost": "local"
        }
        self.client.post('/simulation', data=json.dumps(load_data))

        self.patch_state = patch('hbp_nrp_backend.simulation_control.__Simulation.Simulation.state')
        self.patch_sm = patch('hbp_nrp_backend.simulation_control.__Simulation.StateMachineInstance')
        self.mock_state = self.patch_state.start()
        self.mock_sm = self.patch_sm.start()
        self.mock_instance = Mock()
        self.mock_sm.return_value = self.mock_instance
        self.mock_instance.sm_id = "Control1"
        simulation = _get_simulation_or_abort(0)
        simulation.state = "paused"

    def tearDown(self):
        del simulations[:]
        self.patch_state.stop()
        self.patch_sm.stop()

    def __put_initialization_side_effect(self, side_effect):
        self.mock_instance.is_running = False
        self.mock_instance.initialize_sm = Mock(side_effect=side_effect)

    def test_simulation_state_machines_get_ok(self):
        response = self.client.get('/simulation/0/state-machines')
        self.assertIsInstance(response, Response)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data.strip(), '{"data": {}}')


    def test_simulation_state_machines_get_ok2(self):
        simulation = _get_simulation_or_abort(0)
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.get('/simulation/0/state-machines')
        self.assertIsInstance(response, Response)
        print("Response data=" + response.data)
        self.assertMultiLineEqual(json.loads(response.data)["data"]["Control1"], SM)
        self.assertEqual(response.status_code, 200)

    def test_simulation_state_machines_put_source_code_error(self):
        simulation = _get_simulation_or_abort(0)
        simulation.set_state_machine_code("Control1", SM)
        self.__put_initialization_side_effect(Exception)
        response = self.client.put('/simulation/0/state-machines/Control1', data="ERROR")
        self.assertIsInstance(response, Response)
        self.assertEqual(response.status_code, 500)

    def test_simulation_state_machines_put_OK(self):
        simulation = _get_simulation_or_abort(0)
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.put('/simulation/0/state-machines/Control1', data=SM)
        self.assertIsInstance(response, Response)
        self.assertEqual(response.status_code, 200)

        simulation.state = "paused"
        response = self.client.put('/simulation/0/state-machines/Control1', data=SM)
        self.assertIsInstance(response, Response)
        self.assertEqual(response.status_code, 200)

    def test_simulation_state_machines_put_wrong_user(self):
        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "wrong-owner"}
        response = self.client.put('/simulation/0/state-machines/Control1', headers=hdr, data=SM)
        self.assertRaises(NRPServicesWrongUserException)
        self.assertEqual(response.status_code, 401)

    def test_simulation_state_machines_put_wrong_state(self):
        simulation = _get_simulation_or_abort(0)
        simulation.state = "bad_state"

        response = self.client.put('/simulation/0/state-machines/Control1', data=SM)
        self.assertEqual(response.status_code, 500)

    def test_simulation_state_machines_put_attribute_error(self):
        simulation = _get_simulation_or_abort(0)
        simulation.set_state_machine_code("Control1", SM)
        self.__put_initialization_side_effect(AttributeError)
        response = self.client.put('/simulation/0/state-machines/Control1', data="X = 1")
        self.assertEqual(response.status_code, 400)

    def test_simulation_state_machines_put_syntax_error(self):
        simulation = _get_simulation_or_abort(0)
        simulation.set_state_machine_code("Control1", SM)
        self.__put_initialization_side_effect(SyntaxError)
        response = self.client.put('/simulation/0/state-machines/Control1', data="X = 1 + .")
        self.assertEqual(response.status_code, 400)

    def test_simulation_state_machines_delete_OK(self):
        simulation = _get_simulation_or_abort(0)
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.delete('/simulation/0/state-machines/Control1')
        self.assertEqual(response.status_code, 200)

    def test_simulation_state_machines_delete_not_found(self):
        simulation = _get_simulation_or_abort(0)
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.delete('/simulation/0/state-machines/Control2')
        self.assertRaises(NRPServicesStateMachineException)
        self.assertEqual(response.status_code, 404)

    def test_simulation_state_machines_delete_wrong_user(self):
        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "wrong-owner"}
        response = self.client.delete('/simulation/0/state-machines/Control1', headers=hdr)
        self.assertRaises(NRPServicesWrongUserException)
        self.assertEqual(response.status_code, 401)


if __name__ == '__main__':
        unittest.main()
