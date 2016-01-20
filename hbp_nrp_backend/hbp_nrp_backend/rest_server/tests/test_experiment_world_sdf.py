"""
Unit tests for the experiment SDF services
"""
__author__ = 'Daniel Peppicelli'

import rospy
from mock import MagicMock, patch, ANY
from hbp_nrp_backend.rest_server.tests import RestTest


class MockServiceResponse:
    def __init__(self):
        pass

    sdf_dump = "<sdf><model name='robot'><somexml/></model></sdf>"


class TestExperimentWorldSDF(RestTest):

    def setUp(self):
        patch_CollabClient = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

    @patch('hbp_nrp_backend.rest_server.__ExperimentWorldSDF.rospy')
    def test_experiment_world_sdf_put(self, mocked_rospy):
        context_id = '123456'
        mocked_rospy.wait_for_service = MagicMock(return_value=None)
        mocked_rospy.ServiceProxy = MagicMock(return_value=MockServiceResponse)
        response = self.client.put('/experiment/' + context_id + '/sdf_world')
        self.assertEqual(response.status_code, 200)
        self.mock_collabClient_instance.replace_file_content_in_collab.assert_called_with(
            "<sdf/>",
            ANY,
            ANY
        )

    @patch('hbp_nrp_backend.rest_server.__ExperimentWorldSDF.rospy')
    def test_experiment_world_sdf_wrong_xml_1(self, mocked_rospy):
        context_id = '123456'
        MockServiceResponse.sdf_dump = "<invalid xml><><><<,,.asdf"
        mocked_rospy.wait_for_service = MagicMock(return_value=None)
        mocked_rospy.ServiceProxy = MagicMock(return_value=MockServiceResponse)
        response = self.client.put('/experiment/' + context_id + '/sdf_world')
        self.assertEqual(response.status_code, 500)

    @patch('hbp_nrp_backend.rest_server.__ExperimentWorldSDF.rospy')
    def test_experiment_world_sdf_wrong_xml_2(self, mocked_rospy):
        context_id = '123456'
        MockServiceResponse.sdf_dump = "<sdf/>"
        mocked_rospy.wait_for_service.side_effect = rospy.ROSException('Mocked ROSException')
        response = self.client.put('/experiment/' + context_id + '/sdf_world')
        self.assertEqual(response.status_code, 500)
