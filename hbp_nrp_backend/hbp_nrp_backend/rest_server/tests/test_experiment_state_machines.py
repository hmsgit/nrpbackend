"""
Unit tests for the service that saves/loads experiment state machines
"""

__author__ = "Bernd Eckstein"

import unittest
import os
import json
import tempfile
import shutil
import mock
from mock import patch, MagicMock, call
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages

from hbp_nrp_commons.generated import exp_conf_api_gen

PATH = os.path.split(__file__)[0]


@patch("hbp_nrp_backend.rest_server.__ExperimentService.get_experiment_basepath")
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

    def test_experiment_collab_state_machines_put_sm_missing(self, mock_bp0):
        mock_bp0.return_value = PATH
        response = self.client.put('/experiment/context_id/state-machines', data={})
        self.assertEqual(response.status_code, 400)

    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
    def test_experiment_collab_state_machines_put_exp_missing(self, collab_mock, mock_bp0):
        client_mock = MagicMock()
        client_mock.clone_file_from_collab_context.return_value = ''
        collab_mock.return_value = client_mock

        mock_bp0.return_value = PATH
        response = self.client.put('/experiment/context_id/state-machines', data=json.dumps({"state_machines":{}}))
        self.assertEqual(response.status_code, 500)

    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
    @patch('hbp_nrp_commons.generated.exp_conf_api_gen.CreateFromDocument')
    def test_experiment_collab_state_machines_put_exp_wrong(self, create_from_mock, collab_mock, mock_bp0):
        client_mock = MagicMock()
        client_mock.clone_file_from_collab_context.return_value = os.path.join(os.path.split(__file__)[0], "ExDConf","test_1.xml")
        collab_mock.return_value = client_mock
        create_from_mock.return_value= ""
        mock_bp0.return_value = PATH

        response = self.client.put('/experiment/context_id/state-machines', data=json.dumps({"state_machines":{}}))
        self.assertEqual(response.status_code, 500)

    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
    def test_experiment_collab_state_machines_put_sm_ok(self, collab_mock, mock_bp0):
        client_mock = MagicMock()
        temp_directory = tempfile.mkdtemp()
        exp_temp_path = os.path.join(temp_directory, "exp_test.xml")
        shutil.copyfile(os.path.join(os.path.split(__file__)[0], "ExDConf","test_1.xml"), exp_temp_path)
        with open(exp_temp_path) as exp_xml:
            exp = exp_conf_api_gen.CreateFromDocument(exp_xml.read())
        client_mock.clone_exp_file_from_collab_context.return_value = exp, exp_temp_path

        collab_mock.return_value = client_mock
        mock_bp0.return_value = PATH
        sm = "def test_sm():\n"\
             "    pass"
        data = {"state_machines":{"test_sm": sm}}

        response = self.client.put('/experiment/context_id/state-machines', data=json.dumps(data))

        self.assertEqual(response.status_code, 200)
        client_mock.write_file_with_content_in_collab.assert_called_with(sm, mock.ANY, 'test_sm.py')
        new_exp = client_mock.replace_file_content_in_collab.call_args_list[0][0][0]
        e = exp_conf_api_gen.CreateFromDocument(new_exp)
        self.assertEqual(len(e.experimentControl.stateMachine), 1)
        for sm in e.experimentControl.stateMachine:
            self.assertEqual(sm.id, 'test_sm')
        if os.path.isdir(temp_directory):
            shutil.rmtree(temp_directory)

if __name__ == '__main__':
    unittest.main()
