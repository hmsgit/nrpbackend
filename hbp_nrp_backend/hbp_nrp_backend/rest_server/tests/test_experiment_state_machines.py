"""
Unit tests for the service that saves/loads experiment state machines
"""

__author__ = "Bernd Eckstein"

import unittest
import os
import json
from mock import patch
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages


PATH = os.getcwd()
if not os.path.exists("ExDConf"):
    PATH += "/hbp_nrp_backend/hbp_nrp_backend/rest_server/tests"


@patch("hbp_nrp_backend.rest_server.__ExperimentService.get_basepath")
class TestExperimentStateMachines(RestTest):

    def test_experiment_state_machines_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/test_sm/state-machines')
        self.assertEqual(response.status_code, 200)
        print response.data

    def test_experiment_state_machines_get_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/__NOT_AVAIABLE__/state-machines')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_state_machines_put_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.put('/experiment/__NOT_AVAIABLE__/state-machines/sm1', data="Test data")
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines.get_control_state_machine_files")
    def test_experiment_state_machines_put_ok(self, mock_get, mock_bp0):
        mock_bp0.return_value = PATH
        mock_get.return_value = dict(sm1=PATH+"/sm_file.py")

        response = self.client.put('/experiment/test_1/state-machines/sm1', data="Test data")
        self.assertEqual(response.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines"
           ".get_evaluation_state_machine_files")
    def test_experiment_state_machines_put_ok2(self, mock_get, mock_bp0):
        mock_bp0.return_value = PATH
        mock_get.return_value = dict(sm1=PATH+"/StateMachines/sm_file.py")

        response = self.client.put('/experiment/test_1/state-machines/sm1', data="Test data")
        self.assertEqual(response.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines"
           ".get_evaluation_state_machine_files")
    def test_experiment_state_machines_put_sm_not_found(self, mock_get, mock_bp0):
        mock_bp0.return_value = PATH
        mock_get.return_value = dict(sm1=PATH+"/StateMachines/sm_file.py")

        response = self.client.put('/experiment/test_1/state-machines/sm2', data="Test data")
        self.assertEqual(response.status_code, 404)
        self.assertDictContainsSubset({"message": "State machine not found: sm2",
                                       "type": "Client error"}, json.loads(response.data))

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines.get_control_state_machine_files")
    def test_experiment_state_machines_get_ok2(self, mock_get, mock_bp0):
        mock_bp0.return_value = PATH
        mock_get.return_value = dict(sm1=PATH+"/StateMachines/sm_file.py")

        response = self.client.get('/experiment/test_1/state-machines', data="Test data")
        self.assertEqual(response.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines.get_control_state_machine_files")
    def test_experiment_state_machines_get_fail(self, mock_get, mock_bp0):
        mock_bp0.return_value = PATH
        mock_get.return_value = dict(sm1=PATH+"/StateMachines/__MISSING_FILE__.py")

        response = self.client.get('/experiment/test_1/state-machines', data="Test data")
        self.assertEqual(response.status_code, 404)


if __name__ == '__main__':
    unittest.main()
