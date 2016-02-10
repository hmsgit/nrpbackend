"""
Unit tests for the __SimulationResetCollab module.
"""

__author__ = 'Alessandro Ambrosano, Ugo Albanese'

import unittest
import mock
import json
from mock import patch, ANY

from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient import NeuroroboticsCollabClient
from hbp_nrp_backend.rest_server.__SimulationResetCollab import SimulationResetCollab
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.simulation_control import simulations, Simulation
from cle_ros_msgs.srv import ResetSimulationRequest


class TestSimulationResetCollab(RestTest):

    def setUp(self):
        patch_CollabClient =\
            patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', None, 'default-owner', 'created'))
        simulations.append(Simulation(1, 'experiment2', None, 'im-not-the-owner', 'created'))

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
        self.assertEqual(401, response.status_code)

        # Now the request is fine, but something goes wrong out of the backend's reach
        simulations[0].cle.reset.side_effect = ROSCLEClientException()
        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        self.assertEqual(500, response.status_code)

    def test_reset_is_called_properly(self):
        simulations[0].cle = mock.MagicMock()

        empty_payload = ""

        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_ROBOT_POSE, empty_payload)

        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_FULL
        }))
        simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_FULL, empty_payload)

        fake_world_sdf_string = '<sdf></sdf>'

        with patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.SimulationResetCollab._compute_payload') \
                as mock_compute_payload:

            mock_compute_payload.return_value = fake_world_sdf_string

            response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_WORLD
            }))
            simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_WORLD, fake_world_sdf_string)

    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.SimulationResetCollab._get_sdf_world_from_collab')
    def test_compute_payload_reset_world(self, mock_get_sdf):

        fake_world_sdf_string = '<sdf></sdf>'
        mock_get_sdf.return_value = fake_world_sdf_string

        from cle_ros_msgs.srv import ResetSimulationRequest

        reset_world_type = ResetSimulationRequest.RESET_WORLD

        self.assertEqual(SimulationResetCollab._compute_payload(reset_world_type, self.context_id),
                         fake_world_sdf_string)

    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.SimulationResetCollab._get_sdf_world_from_collab')
    def test_compute_payload_no_payload(self, mock_get_sdf):

        fake_world_sdf_string = '<sdf></sdf>'
        mock_get_sdf.return_value = fake_world_sdf_string

        from cle_ros_msgs.srv import ResetSimulationRequest

        reset_robot_pose_type = ResetSimulationRequest.RESET_ROBOT_POSE

        empty_payload = ''

        self.assertEqual(SimulationResetCollab._compute_payload(reset_robot_pose_type, self.context_id),
                         empty_payload)

    #UserAuthentication.get_header_token
    # NeuroroboticsCollabClient
    # client.get_first_file_path_with_mimetype
    @patch('hbp_nrp_backend.rest_server.__SimulationResetCollab.UserAuthentication.get_header_token')
    def test_get_sdf_world_from_collab(self, mock_get_header_token):

        fake_filepath = '/abc/cde'
        self.mock_collabClient_instance.get_first_file_path_with_mimetype.return_value = fake_filepath

        fake_world_sdf_string = '<sdf></sdf>'
        self.mock_collabClient_instance.download_file_from_collab.return_value = fake_world_sdf_string

        #restore constant field
        self.mock_CollabClient.SDF_WORLD_MIMETYPE = NeuroroboticsCollabClient.SDF_WORLD_MIMETYPE

        # call target function
        world_sdf_string = SimulationResetCollab._get_sdf_world_from_collab(self.context_id)

        # assertions
        mock_get_header_token.assert_called()

        self.mock_collabClient_instance.get_first_file_path_with_mimetype.assert_called_with(
            NeuroroboticsCollabClient.SDF_WORLD_MIMETYPE, ANY)

        self.mock_collabClient_instance.download_file_from_collab.assert_called_with(fake_filepath)

        self.assertEqual(world_sdf_string, fake_world_sdf_string)

if __name__ == '__main__':
    unittest.main()