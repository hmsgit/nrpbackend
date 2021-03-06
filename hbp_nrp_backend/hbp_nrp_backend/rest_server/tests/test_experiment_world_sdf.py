# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
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
Unit tests for the experiment SDF services
"""
__author__ = 'Daniel Peppicelli'

import rospy
import os
import shutil
import tempfile
import json
from mock import MagicMock, patch
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

class FakeRobot(object):
    def __init__(self,id):
        self.robot_id = id 

class MockedCLE(MagicMock):
    def get_simulation_robots(self):
        return [FakeRobot('fakeID1'),FakeRobot('fakeID1')]

class MockedSimulation:
    experiment_id = False
    cle = MockedCLE()
    
class TestExperimentWorldSDF(RestTest):

    def setUp(self):
        patch_StorageClient = patch('hbp_nrp_backend.rest_server.__WorldSDFService.StorageClient')
        self.addCleanup(patch_StorageClient.stop)
        self.mock_StorageClient = patch_StorageClient.start()
        self.mock_storageClient_instance = self.mock_StorageClient.return_value
        self.test_directory = os.path.split(__file__)[0]
        self.temp_directory = tempfile.mkdtemp()

    @patch('hbp_nrp_backend.rest_server.__WorldSDFService._get_simulation_or_abort')
    @patch('hbp_nrp_backend.rest_server.__WorldSDFService.rospy')
    @patch('hbp_nrp_backend.rest_server.__SimulationRobot')
    def test_experiment_world_sdf_put(self,mocked_get_sim_robots, mocked_rospy, path_get_sim):
        sim = MockedSimulation()
        sim.experiment_id = '123456'
        path_get_sim.return_value = sim

        mocked_rospy.wait_for_service = MagicMock(return_value=None)
        mocked_rospy.ServiceProxy = MagicMock(return_value=MockServiceResponse)

        exd_conf_original_path = os.path.join(self.test_directory, "experiments", "experiment_data","test_1.exc")
        exd_conf_temp_path = os.path.join(self.temp_directory, "bibi_test.xml")
        shutil.copyfile(exd_conf_original_path, exd_conf_temp_path)
        with open(exd_conf_temp_path) as exp_xml:
            exp = exp_conf_api_gen.CreateFromDocument(exp_xml.read())
        def empty(a,b,c,d,e):
            return
        self.mock_storageClient_instance.clone_file.return_value =  exd_conf_temp_path
        self.mock_storageClient_instance.create_or_update = empty
        self.mock_storageClient_instance.parse_and_check_file_is_valid.return_value =  exp
        response = self.client.post('/simulation/0/sdf_world')
        self.assertEqual(response.status_code, 200)

    @patch('hbp_nrp_backend.rest_server.__WorldSDFService._get_simulation_or_abort')
    @patch('hbp_nrp_backend.rest_server.__WorldSDFService.rospy')
    def test_experiment_world_sdf_put_storage(self, mocked_rospy, path_get_sim):
        sim = MockedSimulation()
        sim.experiment_id = '123456'
        path_get_sim.return_value = sim

        mocked_rospy.wait_for_service = MagicMock(return_value=None)
        mocked_rospy.ServiceProxy = MagicMock(return_value=MockServiceResponse)

        exd_conf_original_path = os.path.join(self.test_directory, "experiments", "experiment_data","test_5.exc")
        exd_conf_temp_path = os.path.join(self.temp_directory, "bibi_test.xml")
        shutil.copyfile(exd_conf_original_path, exd_conf_temp_path)
        with open(exd_conf_temp_path) as exp_xml:
            exp = exp_conf_api_gen.CreateFromDocument(exp_xml.read())
        def empty(a,b,c,d,e):
            return
        self.mock_storageClient_instance.parse_and_check_file_is_valid.return_value = exp
        self.mock_storageClient_instance.clone_file.return_value =  exd_conf_temp_path
        self.mock_storageClient_instance.create_or_update = empty
        response = self.client.post('/simulation/0/sdf_world')
        self.assertEqual(response.status_code, 200)

    @patch('hbp_nrp_backend.rest_server.__WorldSDFService._get_simulation_or_abort')
    @patch('hbp_nrp_backend.rest_server.__WorldSDFService.rospy')
    def test_experiment_world_sdf_wrong_xml_1(self, mocked_rospy, path_get_sim):
        sim = MockedSimulation()
        sim.experiment_id = '123456'
        path_get_sim.return_value = sim

        MockServiceResponse.sdf_dump = "<invalid xml><><><<,,.asdf"
        mocked_rospy.wait_for_service = MagicMock(return_value=None)
        mocked_rospy.ServiceProxy = MagicMock(return_value=MockServiceResponse)
        response = self.client.post('/simulation/0/sdf_world')
        self.assertEqual(response.status_code, 500)

    @patch('hbp_nrp_backend.rest_server.__WorldSDFService._get_simulation_or_abort')
    @patch('hbp_nrp_backend.rest_server.__WorldSDFService.rospy')
    def test_experiment_world_sdf_wrong_xml_2(self, mocked_rospy, path_get_sim):
        sim = MockedSimulation()
        sim.experiment_id = '123456'
        path_get_sim.return_value = sim

        experiment_id = '123456'
        MockServiceResponse.sdf_dump = "<sdf/>"
        mocked_rospy.wait_for_service.side_effect = rospy.ROSException('Mocked ROSException')
        response = self.client.post('/simulation/0/sdf_world')
        self.assertEqual(response.status_code, 500)
