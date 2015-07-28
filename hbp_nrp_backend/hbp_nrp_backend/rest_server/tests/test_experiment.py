"""
Unit tests for the service that retrieves experiments
"""

__author__ = "Bernd Eckstein"

from flask import Response, Request
from hbp_nrp_backend.rest_server import app
from mock import patch, MagicMock, mock_open
from hbp_nrp_backend.rest_server.__ExperimentService import\
    ErrorMessages, get_basepath, save_file, get_username
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
import unittest
import os
import json

PATH = os.getcwd()
if not os.path.exists("ExDConf"):
    PATH += "/hbp_nrp_backend/hbp_nrp_backend/rest_server/tests"

imp1 = "@nrp.MapSpikeSink('left_wheel_neuron', nrp.brain.actors[1], nrp.population_rate)\n" \
       "@nrp.Neuron2Robot(Topic('/monitor/population_rate', cle_ros_msgs.msg.SpikeRate))\n" \
       "def left_wheel_neuron_rate_monitor(t, left_wheel_neuron):\n" \
       "    return cle_ros_msgs.msg.SpikeRate(t, left_wheel_neuron.rate, " \
       "'left_wheel_neuron_rate_monitor')\n"

imp2 = "@nrp.MapSpikeSink('all_neurons', nrp.brain.circuit[slice(0, 8, 1)], " \
       "nrp.spike_recorder)\n" \
       "@nrp.Neuron2Robot(Topic('/monitor/spike_recorder', cle_ros_msgs.msg.SpikeEvent))\n" \
       "def all_neurons_spike_monitor(t, all_neurons):\n" \
       "    return monitoring.create_spike_recorder_message(t, 8, all_neurons.times, " \
       "'all_neurons_spike_monitor')\n"


@patch("hbp_nrp_backend.rest_server.__ExperimentService.get_basepath")
class TestExperimentService(unittest.TestCase):

    # TEST ExperimentService
    def test_experiment_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment')
        assert(isinstance(response, Response))
        self.assertEqual(response.status_code, 200)

    # TEST ExperimentPreview
    def test_experiment_preview_get_exp_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/__NOT_EXISTING__/preview')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentPreview.get_basepath")
    def test_experiment_preview_get_preview_not_found(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/test_2/preview')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_PREVIEW_NOT_FOUND_404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentPreview.get_basepath")
    def test_experiment_preview_get_ok(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/test_1/preview')
        self.assertEqual(response.status_code, 200)

    # Test ExperimentConf
    def test_experiment_conf_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment')
        self.assertEqual(response.status_code, 200)

        data = json.loads(response.data)
        for current in data['data']:
            print current
            response2 = client.get('/experiment/'+current+'/conf')
            self.assertEqual(response2.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.os")
    def test_experiment_conf_get_experiment_file_not_found(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH
        mock_os.path.isfile.return_value = False

        client = app.test_client()
        response = client.get('/experiment/test_1/conf')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_FILE_NOT_FOUND_404)

    def test_experiment_conf_get_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/__NOT_AVAIABLE__/conf')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_conf_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/test_1/conf')
        self.assertEqual(response.status_code, 200)

    def test_experiment_conf_put_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        client = app.test_client()
        response = client.put('/experiment/__NOT_AVAIABLE__/conf', data=json.dumps(data))
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_conf_put_base64_error(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        client = app.test_client()
        response = client.put('/experiment/test_1/conf', data=json.dumps(data))

        self.assertEqual(response.status_code, 400)
        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.ERROR_IN_BASE64_400.format("Incorrect padding"))

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.os")
    def test_experiment_conf_put_ok(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'SGVsbG8gV29ybGQK'}  # "Hello World"
        client = app.test_client()
        response = client.put('/experiment/test_1/conf', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)

    # Test ExperimentBibi
    def test_experiment_bibi_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment')
        self.assertEqual(response.status_code, 200)

        data = json.loads(response.data)
        for current in data['data']:
            print current
            response2 = client.get('/experiment/'+current+'/bibi')
            self.assertEqual(response2.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentBibi.os")
    def test_experiment_bibi_get_experiment_file_not_found(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH
        mock_os.path.isfile.return_value = False

        client = app.test_client()
        response = client.get('/experiment/test_1/bibi')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_FILE_NOT_FOUND_404)

    def test_experiment_bibi_get_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/__NOT_AVAIABLE__/conf')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_bibi_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        client = app.test_client()
        response = client.get('/experiment/test_1/bibi')
        self.assertEqual(response.status_code, 200)

    def test_experiment_bibi_put_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        client = app.test_client()
        response = client.put('/experiment/__NOT_AVAIABLE__/bibi', data=json.dumps(data))
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_bibi_put_base64_error(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        client = app.test_client()
        response = client.put('/experiment/test_1/bibi', data=json.dumps(data))

        self.assertEqual(response.status_code, 400)
        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.ERROR_IN_BASE64_400.format("Incorrect padding"))

    @patch("hbp_nrp_backend.rest_server.__ExperimentBibi.os")
    def test_experiment_bibi_put_ok(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'SGVsbG8gV29ybGQK'}  # "Hello World"
        client = app.test_client()
        response = client.put('/experiment/test_1/bibi', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)

    # Test ExperimentTransferfunctions
    def test_experiment_tf_put_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        client = app.test_client()
        response = client.put('/experiment/__NOT_AVAIABLE__/bibi', data=json.dumps(data))
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentBibi.os")
    def test_experiment_tf_put_ok(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'transfer_functions': [imp1, imp2]}  # "Hello World"
        client = app.test_client()
        response = client.put('/experiment/test_1/transferfunctions', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)


class TestExperimentService2(unittest.TestCase):

    @patch("hbp_nrp_backend.rest_server.__ExperimentService.os")
    def test_get_basepath_ok(self, mock_os):
        mock_os.environ.get.return_value = "/test1"
        self.assertEqual("/test1", get_basepath())

    @patch("hbp_nrp_backend.rest_server.__ExperimentService.os")
    def test_get_basepath_error(self, mock_os):
        mock_os.environ.get.return_value = None
        self.assertRaises(NRPServicesGeneralException, get_basepath)

    @patch("hbp_nrp_backend.rest_server.__ExperimentService.get_basepath")
    def test_save_file(self, mock_basepath):
        mock_basepath.return_value = "/allowed/path"
        self.assertRaises(NRPServicesGeneralException, save_file, "SGVsbG8gV29ybGQK",
                          "/somewhere/else/test.xml")
        # Test break out of allowed path
        self.assertRaises(NRPServicesGeneralException, save_file, "SGVsbG8gV29ybGQK",
                          "/allowed/path/../../broken/out.xml")
        # Test relative path
        self.assertRaises(NRPServicesGeneralException, save_file, "SGVsbG8gV29ybGQK",
                          "../../../relative/path/out.xml")


if __name__ == '__main__':
    unittest.main()
