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
from hbp_nrp_backend.experiment_control import ExperimentStateMachineInstance
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server import NRPServicesGeneralException, \
    NRPServicesStateMachineException, NRPServicesWrongUserException


SM = """\
import mock
initialize_cb = None
def create_state_machine():
    return mock_instance

def __initialize_side_effect(*args, **kwargs):
    global initialize_cb
    initialize_cb = args[0]

mock_instance = mock.Mock()
mock_instance.sm.register_termination_cb.side_effect = __initialize_side_effect
"""

PATH = os.getcwd()
if not os.path.exists("ExDConf"):
    PATH += "/hbp_nrp_backend/hbp_nrp_backend/rest_server/tests"


class TestSimulationStateMachines(RestTest):

    os.environ['NRP_MODELS_DIRECTORY'] = PATH

    def setUp(self):
        load_data = {
            "experimentConfiguration": "ExDConf/test_1.xml",
            "gzserverHost": "local"
        }
        self.client.post('/simulation', data=json.dumps(load_data))

        self.patch_state = patch('hbp_nrp_backend.simulation_control.__Simulation.Simulation.state')
        self.mock_state = self.patch_state.start()
        simulation = _get_simulation_or_abort(0)
        simulation.state = "initialized"

    def tearDown(self):
        del simulations[:]
        self.patch_state.stop()

    def test_simulation_state_machines_get_ok(self):
        response = self.client.get('/simulation/0/state-machines')
        self.assertIsInstance(response, Response)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data.strip(), '{"data": {}}')


    def test_simulation_state_machines_get_ok2(self):
        simulation = _get_simulation_or_abort(0)
        simulation.state_machines["Control1"] = ExperimentStateMachineInstance("Control1")
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.get('/simulation/0/state-machines')
        self.assertIsInstance(response, Response)
        self.assertMultiLineEqual(json.loads(response.data)["data"]["Control1"], SM)
        self.assertEqual(response.status_code, 200)

    def test_simulation_state_machines_put_source_code_error(self):
        simulation = _get_simulation_or_abort(0)
        simulation.state_machines["Control1"] = ExperimentStateMachineInstance("Control1")
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.put('/simulation/0/state-machines/Control1', data="ERROR")
        self.assertIsInstance(response, Response)
        self.assertEqual(response.status_code, 400)

    def test_simulation_state_machines_put_OK(self):
        simulation = _get_simulation_or_abort(0)
        simulation.state_machines["Control1"] = ExperimentStateMachineInstance("Control1")
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
        simulation.state_machines["Control1"] = ExperimentStateMachineInstance("Control1")
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.put('/simulation/0/state-machines/Control1', data="X = 1")
        self.assertEqual(response.status_code, 400)

    def test_simulation_state_machines_put_syntax_error(self):
        simulation = _get_simulation_or_abort(0)
        simulation.state_machines["Control1"] = ExperimentStateMachineInstance("Control1")
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.put('/simulation/0/state-machines/Control1', data="X = 1 + .")
        self.assertEqual(response.status_code, 400)

    def test_simulation_state_machines_delete_OK(self):
        simulation = _get_simulation_or_abort(0)
        simulation.state_machines["Control1"] = ExperimentStateMachineInstance("Control1")
        simulation.set_state_machine_code("Control1", SM)

        response = self.client.delete('/simulation/0/state-machines/Control1')
        self.assertEqual(response.status_code, 200)

    def test_simulation_state_machines_delete_not_found(self):
        simulation = _get_simulation_or_abort(0)
        simulation.state_machines["Control1"] = ExperimentStateMachineInstance("Control1")
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
