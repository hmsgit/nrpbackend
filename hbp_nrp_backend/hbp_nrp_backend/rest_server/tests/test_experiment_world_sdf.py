"""
Unit tests for the experiment SDF services
"""
__author__ = 'Daniel Peppicelli'

import rospy
import os
import shutil
import tempfile
from mock import MagicMock, patch, ANY
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_commons.generated import exp_conf_api_gen

class MockServiceResponse:
    def __init__(self):
        pass

    sdf_dump = """
    <sdf version="1.5">
    <model name="robot">THIS SHOULD NOT BE SAVED</model>
    <world name="default">
    <state world_name="default">
      <sim_time>5 180000000</sim_time>
      <real_time>0 -2434222</real_time>
      <wall_time>1455272988 110239779</wall_time>
      <model name="plane">
        <pose>0 0 0 0 -0 0</pose>
        <link name="link">
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name="robot">
        <pose>1 2 3 0 0 0</pose>
      </model>
    </state>
    </world>
    </sdf>
    """


class TestExperimentWorldSDF(RestTest):

    def setUp(self):
        patch_CollabClient = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value
        self.test_directory = os.path.split(__file__)[0]
        self.temp_directory = tempfile.mkdtemp()

    @patch('hbp_nrp_backend.rest_server.__ExperimentWorldSDF.rospy')
    def test_experiment_world_sdf_put(self, mocked_rospy):
        context_id = '123456'
        mocked_rospy.wait_for_service = MagicMock(return_value=None)
        mocked_rospy.ServiceProxy = MagicMock(return_value=MockServiceResponse)

        exd_conf_original_path = os.path.join(self.test_directory, "ExDConf","test_1.xml")
        exd_conf_temp_path = os.path.join(self.temp_directory, "bibi_test.xml")
        shutil.copyfile(exd_conf_original_path, exd_conf_temp_path)
        self.mock_collabClient_instance.clone_file_from_collab_context.return_value = exd_conf_temp_path

        response = self.client.put('/experiment/' + context_id + '/sdf_world')
        self.assertEqual(response.status_code, 200)
        arg1, arg2, arg3, arg4 = self.mock_collabClient_instance.replace_file_content_in_collab.call_args_list[0][0]
        self.assertFalse("THIS SHOULD NOT BE SAVED" in arg1)
        self.assertEqual(arg4, "recovered_world.sdf")

        arg1, arg2, arg3, arg4 = self.mock_collabClient_instance.replace_file_content_in_collab.call_args_list[1][0]
        experiment_configuration = exp_conf_api_gen.CreateFromDocument(arg1)
        self.assertEqual(experiment_configuration.environmentModel.robotPose.x, 1)
        self.assertEqual(experiment_configuration.environmentModel.robotPose.y, 2)
        self.assertEqual(experiment_configuration.environmentModel.robotPose.z, 3)
        self.assertEqual(experiment_configuration.environmentModel.robotPose.ux, 0.)
        self.assertEqual(experiment_configuration.environmentModel.robotPose.uy, 0.)
        self.assertEqual(experiment_configuration.environmentModel.robotPose.uz, 0.)
        self.assertEqual(experiment_configuration.environmentModel.robotPose.theta, 1.)
        self.assertEqual(arg4, "recovered_experiment_configuration.xml")


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
