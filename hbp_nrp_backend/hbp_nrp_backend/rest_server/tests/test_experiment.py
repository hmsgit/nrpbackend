"""
Unit tests for the service that retrieves experiments
"""

__author__ = "Bernd Eckstein"

import unittest
import os
import sys
import json
from flask import Response, Request
from mock import patch
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server.__ExperimentService import \
    ErrorMessages, get_basepath, save_file, \
    get_control_state_machine_files, get_evaluation_state_machine_files
from hbp_nrp_backend.rest_server import NRPServicesGeneralException


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


class DevNull(object):
    def write(self, data):
        pass


@patch("hbp_nrp_backend.rest_server.__ExperimentService.get_basepath")
class TestExperimentService(RestTest):

    def setUp(self):
        self.old_stderr = sys.stderr
        sys.stderr = DevNull()

    def tearDown(self):
        sys.stderr = self.old_stderr

    # TEST ExperimentService
    def test_experiment_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment')
        assert(isinstance(response, Response))
        self.assertEqual(response.status_code, 200)

    # TEST ExperimentPreview
    def test_experiment_preview_get_exp_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/__NOT_EXISTING__/preview')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentPreview.get_basepath")
    def test_experiment_preview_get_preview_not_found(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        response = self.client.get('/experiment/test_2/preview')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_PREVIEW_NOT_FOUND_404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentPreview.get_basepath")
    def test_experiment_preview_get_ok(self, mock_bp1, mock_bp0):
        mock_bp0.return_value = PATH
        mock_bp1.return_value = PATH

        response = self.client.get('/experiment/test_1/preview')
        self.assertEqual(response.status_code, 200)

    # Test ExperimentConf
    def test_experiment_conf_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment')
        self.assertEqual(response.status_code, 200)

        data = json.loads(response.data)
        for current in data['data']:
            response2 = self.client.get('/experiment/'+current+'/conf')
            self.assertEqual(response2.status_code, 200)

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.os")
    def test_experiment_conf_get_experiment_file_not_found(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH
        mock_os.path.isfile.return_value = False

        response = self.client.get('/experiment/test_1/conf')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_CONF_FILE_NOT_FOUND_404)

    def test_experiment_conf_get_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/__NOT_AVAIABLE__/conf')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_conf_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/test_1/conf')
        self.assertEqual(response.status_code, 200)

    def test_experiment_conf_put_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        response = self.client.put('/experiment/__NOT_AVAIABLE__/conf', data=json.dumps(data))
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_conf_put_base64_error(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        response = self.client.put('/experiment/test_1/conf', data=json.dumps(data))

        self.assertEqual(response.status_code, 400)
        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.ERROR_IN_BASE64_400.format("Incorrect padding"))

    @patch("hbp_nrp_backend.rest_server.__ExperimentConf.os")
    def test_experiment_conf_put_ok(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'SGVsbG8gV29ybGQK'}  # "Hello World"
        response = self.client.put('/experiment/test_1/conf', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)

    # Test ExperimentBibi
    @patch("hbp_nrp_backend.rest_server.__ExperimentBibi.os")
    def test_experiment_bibi_get_experiment_file_not_found(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH
        mock_os.path.isfile.return_value = False

        response = self.client.get('/experiment/test_1/bibi')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_BIBI_FILE_NOT_FOUND_404)

    def test_experiment_bibi_get_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/__NOT_AVAIABLE__/conf')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_bibi_get_ok(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/test_1/bibi')
        self.assertEqual(response.status_code, 200)

    def test_experiment_bibi_put_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        response = self.client.put('/experiment/__NOT_AVAIABLE__/bibi', data=json.dumps(data))
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_bibi_put_base64_error(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'X'}
        response = self.client.put('/experiment/test_1/bibi', data=json.dumps(data))

        self.assertEqual(response.status_code, 400)
        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.ERROR_IN_BASE64_400.format("Incorrect padding"))

    @patch("hbp_nrp_backend.rest_server.__ExperimentBibi.os")
    def test_experiment_bibi_put_ok(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'base64': 'SGVsbG8gV29ybGQK'}  # "Hello World"
        response = self.client.put('/experiment/test_1/bibi', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)

    # Test ExperimentTransferfunctions
    @patch("hbp_nrp_backend.rest_server.__ExperimentBibi.os")
    def test_experiment_tf_get_ok(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/test_1/transfer-functions')
        self.assertEqual(response.status_code, 200)

        tf_dict = json.loads(response.get_data())['data']
        self.assertEqual(len(tf_dict), 4)
        for tf in tf_dict:
            self.assertIn("@nrp.", tf_dict[tf])

    def test_experiment_tf_get_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/__NOT_AVAIABLE__/transfer-functions')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    @patch("hbp_nrp_backend.rest_server.__ExperimentBibi.os")
    def test_experiment_tf_put_ok(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'transfer_functions': [imp1, imp2]}  # "Hello World"
        response = self.client.put('/experiment/test_1/transfer-functions', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)

    def test_experiment_tf_put_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        data = {'transfer_functions': [imp1, imp2]}  # "Hello World"
        response = self.client.put('/experiment/__NOT_AVAIABLE__/transfer-functions',
                              data=json.dumps(data))
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    # Test State Machine
    def test_get_control_state_machine_files(self, mock_bp0):
        mock_bp0.return_value = PATH

        files = get_control_state_machine_files("test_sm")
        self.assertEqual(len(files), 1)

        files = get_control_state_machine_files("test_1")
        self.assertEqual(len(files), 0)

    def test_get_evaluation_state_machine_files(self, mock_bp0):
        mock_bp0.return_value = PATH

        files = get_evaluation_state_machine_files("test_sm")
        self.assertEqual(len(files), 1)

        files = get_evaluation_state_machine_files("test_1")
        self.assertEqual(len(files), 0)

    # Test Brain Call
    @patch("hbp_nrp_backend.rest_server.__ExperimentBibi.os")
    def test_experiment_brain_get_brain_file_not_found(self, mock_os, mock_bp0):
        mock_bp0.return_value = PATH
        mock_os.path.isfile.return_value = False

        response = self.client.get('/experiment/test_3/brain')
        self.assertEqual(response.status_code, 500)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_BRAIN_FILE_NOT_FOUND_500)

    def test_experiment_brain_get_experiment_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/__NOT_AVAIABLE__/brain')
        self.assertEqual(response.status_code, 404)

        message = json.loads(response.get_data())['message']
        self.assertEqual(message, ErrorMessages.EXPERIMENT_NOT_FOUND_404)

    def test_experiment_brain_get_bibi_not_found(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/test_4/brain')

        self.assertEqual(response.status_code, 500)

    def test_experiment_brain_get_ok_h5(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/test_1/brain')
        self.assertEqual(response.status_code, 200)

        data = json.loads(response.get_data())
        self.assertEqual("h5", data['brain_type'])
        self.assertEqual("base64", data['data_type'])
        self.assertIn("iUhERg0KGgoAAAAAAAgIAAQAEAAAAAAAAAAAAAAAAAD//////////9g6AAAAAAAA",
                      data['data'], "The data does not contain the expected substring.")

    def test_experiment_brain_get_ok_py(self, mock_bp0):
        mock_bp0.return_value = PATH

        response = self.client.get('/experiment/test_2/brain')
        self.assertEqual(response.status_code, 200)

        data = json.loads(response.get_data())
        self.assertEqual("py", data['brain_type'])
        self.assertEqual("text", data['data_type'])
        self.assertIn("This file contains the setup of the neuronal network running the Husky",
                      data['data'], "The data does not contain the expected substring.")


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
