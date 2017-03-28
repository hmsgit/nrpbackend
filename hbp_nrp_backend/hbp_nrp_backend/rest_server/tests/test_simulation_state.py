"""
Tests the simulation state service
"""

from flask import Response
from mock import patch, Mock, PropertyMock
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.simulation_control import simulations
import json
from transitions import MachineError

__author__ = "Georg Hinkel"

class TestSimulationStateService(RestTest):

    def setUp(self):
        load_data = {
            "experimentConfiguration": "experiments/experiment_data/test_1.exc",
            "gzserverHost": "local"
        }
        self.client.post('/simulation', data=json.dumps(load_data))

        self.patch_state = patch('hbp_nrp_backend.simulation_control.__Simulation.Simulation.state', new_callable=PropertyMock)
        self.mock_state = self.patch_state.start()

    def tearDown(self):
        del simulations[:]
        self.patch_state.stop()

    def test_get_state(self):
        self.mock_state.return_value = "foobar"
        response = self.client.get('/simulation/0/state')
        assert(isinstance(response, Response))
        self.assertEqual(response.status_code, 200)
        self.assertEqual({"state": "foobar"}, json.loads(response.data))

    def test_set_state_ok(self):
        self.mock_state.return_value = "foo"
        response = self.client.put('/simulation/0/state', data='{"state": "bar"}')
        assert (isinstance(response, Response))
        self.assertEqual(response.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__SimulationState.UserAuthentication")
    def test_set_state_wrong_user(self, user_auth):
        user_auth.matches_x_user_name_header.return_value = False
        self.mock_state.return_value = "foo"
        response = self.client.put('/simulation/0/state', data='{"state": "bar"}')
        assert (isinstance(response, Response))
        self.assertEqual(response.status_code, 401)

    def test_set_state_invalid_transition(self):
        self.mock_state.return_value = "foo"
        def create_machine_error(arg=None):
            if arg:
                self.assertEqual("bar", arg)
                raise MachineError("foo->bar")
            else:
                return "foo"
        self.mock_state.side_effect = create_machine_error
        response = self.client.put('/simulation/0/state', data='{"state": "bar"}')
        assert (isinstance(response, Response))
        self.assertEqual(response.status_code, 400)
