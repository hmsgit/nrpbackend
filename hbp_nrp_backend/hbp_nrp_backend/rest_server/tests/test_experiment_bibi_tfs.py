__author__ = 'Georg Hinkel'


import unittest
import os
import json
from mock import patch
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages


PATH = os.path.split(__file__)[0]
EXPERIMENTS_PATH = os.path.join(PATH, 'experiments')

@patch("hbp_nrp_backend.rest_server.__ExperimentService.get_experiment_basepath")
class TestExperimentBIBITransferFunctions(RestTest):

    def test_experiment_state_machines_get_ok(self, mock_bp0):
        mock_bp0.return_value = EXPERIMENTS_PATH

        response = self.client.get('/experiment/test_1/bibi-transfer-functions')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data,
                         '{"transferFunctions": ['
                         '{"body": '
                         '"return geometry_msgs.msg.Twist('
                         'linear=geometry_msgs.msg.Vector3(x=20.0 * min(left_wheel_neuron.voltage, right_wheel_neuron.voltage), y=0.0, z=0.0), '
                         'angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=100.0 * (right_wheel_neuron.voltage - left_wheel_neuron.voltage)))", '
                         '"topics": ['
                         '{"topic": "/husky/cmd_vel", "parameterName": "__return__", "type": "publisher", "topicType": "geometry_msgs.msg.Twist"}'
                         '], '
                         '"name": "linear_twist", '
                         '"devices": ['
                         '{"parameterName": "left_wheel_neuron", "neurons": "actors[1]", "type": "LeakyIntegratorAlpha"}, '
                         '{"parameterName": "right_wheel_neuron", "neurons": "actors[2]", "type": "LeakyIntegratorAlpha"}'
                         ']}, '
                         '{"body": '
                         '"image_results = hbp_nrp_cle.tf_framework.tf_lib.detect_red(image=camera.value)\\n'
                         'red_left_eye.rate = 1000.0 * image_results.left\\n'
                         'red_right_eye.rate = 1000.0 * image_results.right\\n'
                         'green_blue_eye.rate = 1000.0 * image_results.go_on\\n", '
                         '"topics": ['
                         '{"topic": "/husky/camera", "parameterName": "camera", "type": "subscriber", "topicType": "sensor_msgs.msg.Image"}'
                         '], '
                         '"name": "eye_sensor_transmit", '
                         '"devices": ['
                         '{"parameterName": "red_left_eye", "neurons": "sensors[0:2:3]", "type": "Poisson"}, '
                         '{"parameterName": "red_right_eye", "neurons": "sensors[1:2:4]", "type": "Poisson"}, '
                         '{"parameterName": "green_blue_eye", "neurons": "sensors[4]", "type": "Poisson"}]}'
                         ']}\n')


if __name__ == '__main__':
    unittest.main()

