# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
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
EXPERIMENTS_PATH = os.path.join(PATH, 'experiments')

@patch("hbp_nrp_backend.rest_server.__ExperimentService.get_experiment_basepath")
class TestExperimentStateMachines(RestTest):

    def test_experiment_state_machines_get_ok(self, mock_bp0):
        mock_bp0.return_value = EXPERIMENTS_PATH

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
        mock_bp0.return_value = EXPERIMENTS_PATH
        mock_get.return_value = dict(sm1=EXPERIMENTS_PATH+"/sm_file.py")

        response = self.client.put('/experiment/test_1/state-machines/sm1', data="Test data")
        self.assertEqual(response.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines"
           ".get_evaluation_state_machine_files")
    def test_experiment_state_machines_put_ok2(self, mock_get, mock_bp0):
        mock_bp0.return_value = EXPERIMENTS_PATH
        mock_get.return_value = dict(sm1=os.path.join(EXPERIMENTS_PATH, "experiment_data/sm_file.py"))

        response = self.client.put('/experiment/test_1/state-machines/sm1', data="Test data")
        self.assertEqual(response.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines"
           ".get_evaluation_state_machine_files")
    def test_experiment_state_machines_put_sm_not_found(self, mock_get, mock_bp0):
        mock_bp0.return_value = EXPERIMENTS_PATH
        mock_get.return_value = dict(sm1=os.path.join(EXPERIMENTS_PATH, "experiment_data/sm_file.py"))

        response = self.client.put('/experiment/test_1/state-machines/sm2', data="Test data")
        self.assertEqual(response.status_code, 404)
        self.assertDictContainsSubset({"message": "State machine not found: sm2",
                                       "type": "Client error"}, json.loads(response.data))

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines.get_control_state_machine_files")
    def test_experiment_state_machines_get_ok2(self, mock_get, mock_bp0):
        mock_bp0.return_value = EXPERIMENTS_PATH
        mock_get.return_value = dict(sm1=os.path.join(EXPERIMENTS_PATH, "experiment_data/sm_file.py"))

        response = self.client.get('/experiment/test_1/state-machines', data="Test data")
        self.assertEqual(response.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentStateMachines.get_control_state_machine_files")
    def test_experiment_state_machines_get_fail(self, mock_get, mock_bp0):
        mock_bp0.return_value = EXPERIMENTS_PATH
        mock_get.return_value = dict(sm1=EXPERIMENTS_PATH+"/StateMachines/__MISSING_FILE__.py")

        response = self.client.get('/experiment/test_1/state-machines', data="Test data")
        self.assertEqual(response.status_code, 404)

    def test_experiment_collab_state_machines_put_sm_missing(self, mock_bp0):
        mock_bp0.return_value = EXPERIMENTS_PATH
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
        client_mock.clone_file_from_collab_context.return_value = os.path.join(os.path.split(__file__)[0], "experiments", "experiment_data","test_1.exc")
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
        exp_remote_path = os.path.join("collab_path", "exp_test.xml")
        shutil.copyfile(os.path.join(os.path.split(__file__)[0], "experiments", "experiment_data","test_1.exc"), exp_temp_path)
        with open(exp_temp_path) as exp_xml:
            exp = exp_conf_api_gen.CreateFromDocument(exp_xml.read())
        client_mock.clone_exp_file_from_collab_context.return_value = exp, exp_temp_path, exp_remote_path
        collab_mock.return_value = client_mock
        mock_bp0.return_value = PATH
        sm = "def test_sm():\n"\
             "    pass"
        data = {"state_machines":{"test_sm": sm}}

        response = self.client.put('/experiment/context_id/state-machines', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)
        client_mock.replace_file_content_in_collab.assert_any_call(content=sm, mimetype=client_mock.STATE_MACHINE_PY_MIMETYPE, filename="test_sm.exd")
        new_exp = client_mock.replace_file_content_in_collab.call_args_list[-1][0][0]
        e = exp_conf_api_gen.CreateFromDocument(new_exp)
        self.assertEqual(len(e.experimentControl.stateMachine), 1)
        for sm in e.experimentControl.stateMachine:
            self.assertEqual(sm.id, 'test_sm')
        if os.path.isdir(temp_directory):
            shutil.rmtree(temp_directory)

if __name__ == '__main__':
    unittest.main()
