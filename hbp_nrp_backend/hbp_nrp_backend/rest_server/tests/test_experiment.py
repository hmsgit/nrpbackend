"""
Unit tests for the service that retrieves experiments
"""

__author__ = "Bernd Eckstein"

from flask import Response, Request
from hbp_nrp_backend.rest_server import app
from mock import patch, MagicMock, mock_open
import unittest
import os
import json

PATH = "./hbp_nrp_backend/hbp_nrp_backend/rest_server/tests"

EXPERIMENT_DICT = {
    "NeuronalRedDetection_Husky": {
        "description": "No description available for this experiment.",
        "experimentConfiguration": "ExDConf/NeuronalRedDetection_Husky.xml",
        "name": "NeuronalRedDetection_Husky",
        "timeout": 600
    }
}


#@patch("hbp_nrp_backend.rest_server.__ExperimentService.get_experiments")
@patch("hbp_nrp_backend.rest_server.__ExperimentService.get_basepath")
class TestExperimentService(unittest.TestCase):

    # TEST ExperimentService
    def test_experiment_get(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment')
        assert(isinstance(response, Response))
        self.assertEqual(response.status_code, 200)

    # TEST ExperimentPreview
    def test_experiment_preview_get_404(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/__NOT_EXISTING__/preview')
        self.assertEqual(response.status_code, 404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentPreview.get_basepath")
    def test_experiment_preview_get_401(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/test_2/preview')
        self.assertEqual(response.status_code, 401)

    @patch("hbp_nrp_backend.rest_server.__ExperimentPreview.get_basepath")
    def test_experiment_preview_get_200(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/test_1/preview')
        self.assertEqual(response.status_code, 200)

    # Test ExperimentConf
    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.get_basepath")
    def test_experiment_conf_get_200(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH
        client = app.test_client()
        response = client.get('/experiment')
        self.assertEqual(response.status_code, 200)
        assert isinstance(response, Response)

        data = json.loads(response.data)
        for current in data['data']:
            print current
            response2 = client.get('/experiment/'+current+'/conf')
            self.assertEqual(response2.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.get_basepath")
    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.os")
    def test_experiment_conf_get_401(self, mock_os, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        mock_os.path.isfile.return_value = False

        client = app.test_client()
        response = client.get('/experiment/test_1/conf')
        self.assertEqual(response.status_code, 401)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.get_basepath")
    def test_experiment_conf_get_404(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/__NOT_AVAIABLE__/conf')
        self.assertEqual(response.status_code, 404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.get_basepath")
    def test_experiment_conf_get_200(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/test_1/conf')
        self.assertEqual(response.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.get_basepath")
    def test_experiment_conf_put_404(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        data = {'base64': 'X'}
        client = app.test_client()
        response = client.put('/experiment/__NOT_AVAIABLE__/conf',
                              data=json.dumps(data))
        self.assertEqual(response.status_code, 404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.get_basepath")
    def test_experiment_conf_put_401(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        data = {'base64': 'X'}
        client = app.test_client()
        response = client.put('/experiment/test_1/conf', data=json.dumps(data))
        self.assertEqual(response.status_code, 401)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.get_basepath")
    def test_experiment_conf_put_401(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        data = {'base64': 'X'}
        client = app.test_client()
        response = client.put('/experiment/test_1/conf', data=json.dumps(data))
        self.assertEqual(response.status_code, 401)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.get_basepath")
    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.os")
    def test_experiment_conf_put_200(self, mock_os, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        data = {'base64': 'SGVsbG8gV29ybGQK'}  # "Hello World"
        client = app.test_client()
        response = client.put('/experiment/test_1/conf', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)


if __name__ == '__main__':
    unittest.main()
