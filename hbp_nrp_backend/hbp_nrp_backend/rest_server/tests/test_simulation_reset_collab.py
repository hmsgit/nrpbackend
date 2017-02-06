"""
Unit tests for the __SimulationResetCollab module.
"""

__author__ = 'Alessandro Ambrosano, Ugo Albanese, Georg Hinkel'

import unittest
import time
import mock
from mock import mock_open
import json
from mock import patch, ANY
import os

from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient import NeuroroboticsCollabClient
from hbp_nrp_backend.rest_server.__SimulationResetCollab import SimulationResetCollab
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.simulation_control import simulations, Simulation
from cle_ros_msgs.srv import ResetSimulationRequest
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from pyxb import ValidationError, NamespaceError


PATH = os.path.split(__file__)[0]
EXPERIMENT_DATA_PATH = os.path.join(PATH, 'experiments', 'experiment_data')

class TestSimulationResetCollab(RestTest):

    def setUp(self):
        patch_CollabClient =\
            patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

        del simulations[:]
        simulations.append(Simulation(0, 'experiments/experiment_data/test_1.exc', None, 'default-owner', 'created'))
        simulations.append(Simulation(1, 'experiments/experiment_data/test_1.exc', None, 'im-not-the-owner', 'created'))

        self.context_id = "0000-0000"
        # Correct request
        self.correct_reset_url = '/simulation/0/' + self.context_id + '/reset'

    def test_put_reset(self):
        simulations[0].cle = mock.MagicMock()

        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        self.assertEqual(200, response.status_code)
        simulations[0].cle.reset.assert_called()

        # Invalid request, too many parameters
        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE,
            'randomInvalidParameter': False
        }))
        self.assertEqual(400, response.status_code)

        # Invalid request, missing parameters
        response = self.client.put(self.correct_reset_url, data=json.dumps({}))
        self.assertEqual(400, response.status_code)

        # This simulation doesn't exist
        response = self.client.put('/simulation/2/' + self.context_id + '/reset')
        self.assertEqual(404, response.status_code)

        # I'm not the owner of this one
        response = self.client.put('/simulation/1/' + self.context_id + '/reset')
        self.assertEqual(403, response.status_code)

        # Now the request is fine, but something goes wrong out of the backend's reach
        simulations[0].cle.reset.side_effect = ROSCLEClientException()
        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        self.assertEqual(500, response.status_code)

    @patch("hbp_nrp_backend.rest_server.__SimulationResetCollab.SimulationResetCollab._get_brain_info_from_collab")
    @patch("hbp_nrp_backend.rest_server.__SimulationResetCollab.copy_tree")
    @patch("hbp_nrp_backend.rest_server.__SimulationResetCollab.get_experiment_data")
    @patch("hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient")
    @patch("hbp_nrp_backend.simulation_control.__Simulation.Simulation.lifecycle")
    def test_reset_is_called_properly(self, mock_lifecycle, mock_neurorobotics_collab_client, \
                                                mock_get_experiment_data, mock_copy_tree, \
                                                mock_get_brain_info_from_collab):
        simulations[0].cle = mock.MagicMock()
        simulations[0].cle.set_simulation_transfer_function.return_value = None

        mock_get_brain_info_from_collab.return_value = os.path.join(PATH,
                                                    'models/braitenberg.py'), {}, {}

        experiment_file_path = os.path.join(PATH, 'experiments/experiment_data/test_1.exc')

        with open(experiment_file_path) as exd_file:
            try:
                experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())
            except ValidationError, ve:
                raise Exception("Could not parse experiment configuration {0:s} due to validation "
                                "error: {1:s}".format(experiment_file_path, str(ve)))

            bibi_file = experiment.bibiConf.src
            bibi_file_abs = os.path.join(EXPERIMENT_DATA_PATH, bibi_file)
            with open(bibi_file_abs) as b_file:
                try:
                    bibi = bibi_api_gen.CreateFromDocument(b_file.read())
                except ValidationError, ve:
                    raise Exception("Could not parse brain configuration {0:s} due to validation "
                                    "error: {1:s}".format(bibi_file_abs, str(ve)))

        mock_get_experiment_data.return_value = experiment, bibi

        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        self.assertEqual(200, response.status_code)
        simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_ROBOT_POSE,
                                                    world_sdf=None,
                                                    brain_path=None,
                                                    populations=None)


        mock_lifecycle.return_value = mock.MagicMock(experiment_path="")
        mock_neurorobotics_collab_client.clone_experiment_template_from_collab_context.return_value = {'experiment_conf': ""}
        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_FULL
        }))

        self.assertEqual(200, response.status_code)
        simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_FULL,
                                                    world_sdf=None,
                                                    brain_path=None,
                                                    populations=None)


        fake_world_sdf_string = '<sdf></sdf>'

        with patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.SimulationResetCollab._compute_payload') \
                as mock_compute_payload:

            mock_compute_payload.return_value = fake_world_sdf_string, None, None

            response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_WORLD
            }))
            simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_WORLD,
                                                        world_sdf=fake_world_sdf_string,
                                                        brain_path=None,
                                                        populations=None)

    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.SimulationResetCollab._get_sdf_world_from_collab')
    def test_compute_payload_reset_world(self, mock_get_sdf):

        fake_world_sdf_string = '<sdf></sdf>'
        mock_get_sdf.return_value = fake_world_sdf_string


        reset_world_type = ResetSimulationRequest.RESET_WORLD

        self.assertEqual(SimulationResetCollab._compute_payload(reset_world_type, self.context_id),
                         (fake_world_sdf_string, None, None))

    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.SimulationResetCollab._get_sdf_world_from_collab')
    def test_compute_payload_no_payload(self, mock_get_sdf):

        fake_world_sdf_string = '<sdf></sdf>'
        mock_get_sdf.return_value = fake_world_sdf_string


        reset_robot_pose_type = ResetSimulationRequest.RESET_ROBOT_POSE

        empty_payload = (None, None, None)

        self.assertEqual(SimulationResetCollab._compute_payload(reset_robot_pose_type, self.context_id),
                         empty_payload)

    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.tempfile')
    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.get_all_neurons_as_dict')
    @patch('hbp_nrp_commons.generated.bibi_api_gen.CreateFromDocument')
    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.UserAuthentication.get_header_token')
    def test_compute_payload_reset_brain(self, mock_get_brain_info, mock_create_from_document,
        mock_get_all_neurons_as_dict, mock_tempfile):

        dummy_dir = "/my/temp/dir"
        mock_tempfile.mkdtemp.return_value = dummy_dir

        self.mock_collabClient_instance.get_first_file_path_with_mimetype.return_value = dummy_dir

        self.mock_collabClient_instance.clone_file_from_collab_context.return_value = os.path.join(dummy_dir, "brain.py"), None
        # restore constant field
        self.mock_CollabClient.BRAIN_PYNN_MIMETYPE = NeuroroboticsCollabClient.BRAIN_PYNN_MIMETYPE
        self.mock_CollabClient.BIBI_CONFIGURATION_MIMETYPE = NeuroroboticsCollabClient.BIBI_CONFIGURATION_MIMETYPE
        self.mock_CollabClient.BIBI_CONFIGURATION_FILE_NAME = NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME

        dummy_populations = {}
        mock_get_all_neurons_as_dict.return_value = dummy_populations

        payload = SimulationResetCollab._compute_payload(ResetSimulationRequest.RESET_BRAIN, self.context_id)
        self.assertEqual(mock_get_brain_info.call_count, 1)
        self.assertEqual(payload, (None,  '/my/temp/dir/brain.py', []))

    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.UserAuthentication.get_header_token')
    def test_get_sdf_world_from_collab(self, mock_get_header_token):

        fake_filepath = '/abc/cde'
        self.mock_collabClient_instance.get_path_by_id.return_value = fake_filepath
        fake_world_sdf_string = '<sdf></sdf>'

        self.mock_collabClient_instance.download_file_from_collab.return_value = fake_world_sdf_string

        # restore constant field
        self.mock_CollabClient.SDF_WORLD_MIMETYPE = NeuroroboticsCollabClient.SDF_WORLD_MIMETYPE
        self.mock_CollabClient.SDF_WORLD_FILE_NAME = NeuroroboticsCollabClient.SDF_WORLD_FILE_NAME
        with patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.open', mock_open(read_data=fake_world_sdf_string), create=True) as m:
            # call target function
            world_sdf_string = SimulationResetCollab._get_sdf_world_from_collab(self.context_id)

        # assertions
        mock_get_header_token.assert_called()

        self.mock_collabClient_instance.clone_file_from_collab_context.assert_called_with(
            NeuroroboticsCollabClient.SDF_WORLD_MIMETYPE, NeuroroboticsCollabClient.SDF_WORLD_FILE_NAME)

        self.assertEqual(world_sdf_string, fake_world_sdf_string)

    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.tempfile')
    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.get_all_neurons_as_dict')
    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.UserAuthentication.get_header_token')
    def test_get_brain_info_from_collab(self, mock_get_brain_info,
        mock_get_all_neurons_as_dict, mock_tempfile):

        dummy_dir = "/my/temp/dir"
        mock_tempfile.mkdtemp.return_value = dummy_dir

        self.mock_collabClient_instance.clone_file_from_collab_context.return_value = os.path.join(dummy_dir, "brain.py"), None
        # restore constant field
        self.mock_CollabClient.BRAIN_PYNN_MIMETYPE = NeuroroboticsCollabClient.BRAIN_PYNN_MIMETYPE
        self.mock_CollabClient.BIBI_CONFIGURATION_MIMETYPE = NeuroroboticsCollabClient.BIBI_CONFIGURATION_MIMETYPE
        self.mock_CollabClient.BIBI_CONFIGURATION_FILE_NAME = NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME

        dummy_populations = {'pop1': slice(0, 1, 1), 'pop2': slice(1, 2, 1)}
        mock_get_all_neurons_as_dict.return_value = dummy_populations
        data_from_collab, populations, _ = SimulationResetCollab._get_brain_info_from_collab(self.context_id)
        self.assertEqual(data_from_collab, os.path.join(dummy_dir, 'brain.py'))
        self.assertEqual("[name: pop2\ntype: 1\nids: []\nstart: 1\nstop: 2\nstep: 1, name: pop1\ntype: 1\nids: []\nstart: 0\nstop: 1\nstep: 1]", str(populations))

if __name__ == '__main__':
    unittest.main()
