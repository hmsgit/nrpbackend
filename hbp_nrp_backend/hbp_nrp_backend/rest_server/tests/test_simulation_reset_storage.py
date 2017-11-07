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
Unit tests for the __SimulationResetStorage module.
"""

__author__ = 'Alessandro Ambrosano, Ugo Albanese, Georg Hinkel'

import unittest
import time
import mock
from mock import mock_open
import json
from mock import patch, ANY
import os
import tempfile
import shutil
from hbp_nrp_backend.storage_client_api import StorageClient
from hbp_nrp_backend.rest_server.__SimulationResetStorage import SimulationResetStorage
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.simulation_control import simulations, Simulation
from cle_ros_msgs.srv import ResetSimulationRequest
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from pyxb import ValidationError, NamespaceError


PATH = os.path.split(__file__)[0]
EXPERIMENT_DATA_PATH = os.path.join(PATH, 'experiments', 'experiment_data')


class TestSimulationResetStorage(RestTest):

    def setUp(self):
        patch_StorageClient =\
            patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient')
        self.addCleanup(patch_StorageClient.stop)
        self.mock_StorageClient = patch_StorageClient.start()
        self.mock_storageClient_instance = self.mock_StorageClient.return_value

        del simulations[:]
        simulations.append(Simulation(
            0, 'experiments/experiment_data/test_1.exc', None, 'default-owner', 'created'))
        simulations.append(Simulation(
            1, 'experiments/experiment_data/test_1.exc', None, 'im-not-the-owner', 'created'))

        self.experiment_id = "0000-0000"
        # Correct request
        self.correct_reset_url = '/simulation/0/' + self.experiment_id + '/reset'

    def test_put_reset(self):
        simulations[0].cle = mock.MagicMock()

        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        self.assertEqual(200, response.status_code)
        simulations[0].cle.reset.assert_called()

    def test_put_reset_too_many_parameters(self):
        # Invalid request, too many parameters
        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE,
            'randomInvalidParameter': False
        }))
        self.assertEqual(400, response.status_code)

    def test_put_reset_missing_parameters(self):
        # Invalid request, missing parameters
        response = self.client.put(self.correct_reset_url, data=json.dumps({}))
        self.assertEqual(400, response.status_code)

    def test_put_reset_sim_not_exist(self):
        # This simulation doesn't exist
        response = self.client.put(
            '/simulation/2/' + self.experiment_id + '/reset')
        self.assertEqual(404, response.status_code)

    def test_put_reset_not_owner(self):
        # I'm not the owner of this one
        response = self.client.put(
            '/simulation/1/' + self.experiment_id + '/reset')
        self.assertEqual(401, response.status_code)

    def test_put_reset_throws(self):
        # Now the request is fine, but something goes wrong out of the
        # backend's reach
        simulations[0].cle = mock.MagicMock()
        simulations[0].cle.reset.side_effect = ROSCLEClientException()
        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        self.assertEqual(500, response.status_code)

    @patch("hbp_nrp_backend.rest_server.__SimulationResetStorage.SimulationResetStorage._get_brain_info_from_storage")
    @patch("hbp_nrp_backend.rest_server.__SimulationResetStorage.copy_tree")
    @patch("hbp_nrp_backend.rest_server.__SimulationResetStorage.get_experiment_data")
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.StorageClient")
    @patch("hbp_nrp_backend.simulation_control.__Simulation.Simulation.lifecycle")
    def test_reset_is_called_properly(self, mock_lifecycle, mock_storage_client,
                                      mock_get_experiment_data, mock_copy_tree,
                                      mock_get_brain_info_from_storage):
        simulations[0].cle = mock.MagicMock()
        simulations[0].cle.set_simulation_transfer_function.return_value = None

        mock_get_brain_info_from_storage.return_value = os.path.join(PATH,
                                                                     'models/braitenberg.py'), {}, {}

        experiment_file_path = os.path.join(
            PATH, 'experiments/experiment_data/test_1.exc')

        with open(experiment_file_path) as exd_file:
            try:
                experiment = exp_conf_api_gen.CreateFromDocument(
                    exd_file.read())
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
        mock_storage_client.clone_all_experiment_files.return_value = None, experiment_file_path
        response = self.client.put(self.correct_reset_url, data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_FULL
        }))

        # this test will fail once the reset functionality is working again (see issue report NRRPLT-4860)
        # at that moment, 500 should be replaced by 200 and RESET_ROBOT_POSE
        # should be replaced by RESET_FULL in the next lines
        self.assertEqual(500, response.status_code)
        simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_ROBOT_POSE,
                                                    world_sdf=None,
                                                    brain_path=None,
                                                    populations=None)

        fake_world_sdf_string = '<sdf></sdf>'

        with patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.SimulationResetStorage._compute_payload') \
                as mock_compute_payload:

            mock_compute_payload.return_value = fake_world_sdf_string, None, None

            response = self.client.put(self.correct_reset_url, data=json.dumps({
                'resetType': ResetSimulationRequest.RESET_WORLD
            }))
            simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_WORLD,
                                                        world_sdf=fake_world_sdf_string,
                                                        brain_path=None,
                                                        populations=None)

    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.SimulationResetStorage._get_sdf_world_from_storage')
    def test_compute_payload_reset_world(self, mock_get_sdf):

        fake_world_sdf_string = '<sdf></sdf>'
        mock_get_sdf.return_value = fake_world_sdf_string

        reset_world_type = ResetSimulationRequest.RESET_WORLD

        self.assertEqual(SimulationResetStorage._compute_payload(reset_world_type, self.experiment_id, None),
                         (fake_world_sdf_string, None, None))

    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.SimulationResetStorage._get_sdf_world_from_storage')
    def test_compute_payload_no_payload(self, mock_get_sdf):

        fake_world_sdf_string = '<sdf></sdf>'
        mock_get_sdf.return_value = fake_world_sdf_string

        reset_robot_pose_type = ResetSimulationRequest.RESET_ROBOT_POSE

        empty_payload = (None, None, None)

        self.assertEqual(SimulationResetStorage._compute_payload(reset_robot_pose_type, self.experiment_id, None),
                         empty_payload)

    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.tempfile')
    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.get_all_neurons_as_dict')
    @patch('hbp_nrp_commons.generated.bibi_api_gen.CreateFromDocument')
    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.UserAuthentication.get_header_token')
    def test_compute_payload_reset_brain(self, mock_get_brain_info, mock_create_from_document,
                                         mock_get_all_neurons_as_dict, mock_tempfile):

        bibi_original_path = os.path.join(os.path.split(
            __file__)[0], "experiments", "experiment_data", "bibi_1.bibi")
        bibi_temp_path = os.path.join(tempfile.mkdtemp(), "bibi_test.xml")
        bibi_remote_path = os.path.join("/collab_dir", "bibi_test.xml")
        shutil.copyfile(bibi_original_path, bibi_temp_path)

        exp_temp_path = os.path.join(os.path.split(
            __file__)[0], "experiments", "experiment_data", "test_1.exc")

        def fake_get(*args, **kwargs):
            if args[2] == 'experiment_configuration.exc':
                with open(exp_temp_path) as exp_xml:
                    return exp_xml.read()
            with open(bibi_temp_path) as bibi_xml:
                return bibi_xml.read()
        self.mock_storageClient_instance.get_file = fake_get

        dummy_dir = "/my/temp/dir"
        mock_tempfile.mkdtemp.return_value = dummy_dir
        self.mock_storageClient_instance.clone_file.return_value = os.path.join(
            dummy_dir, "brain.py")

        dummy_populations = {}
        mock_get_all_neurons_as_dict.return_value = dummy_populations

        payload = SimulationResetStorage._compute_payload(
            ResetSimulationRequest.RESET_BRAIN, self.experiment_id, None)
        self.assertEqual(mock_get_brain_info.call_count, 3)
        self.assertEqual(payload, (None,  '/my/temp/dir/brain.py', []))

    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.UserAuthentication.get_header_token')
    def test_get_sdf_world_from_storage(self, mock_get_header_token):

        bibi_original_path = os.path.join(os.path.split(
            __file__)[0], "experiments", "experiment_data", "bibi_1.bibi")
        bibi_temp_path = os.path.join(tempfile.mkdtemp(), "bibi_test.xml")
        bibi_remote_path = os.path.join("/collab_dir", "bibi_test.xml")
        shutil.copyfile(bibi_original_path, bibi_temp_path)

        exp_temp_path = os.path.join(os.path.split(
            __file__)[0], "experiments", "experiment_data", "test_1.exc")

        def fake_get(*args, **kwargs):
            if args[2] == 'experiment_configuration.exc':
                with open(exp_temp_path) as exp_xml:
                    return exp_xml.read()
            with open(bibi_temp_path) as bibi_xml:
                return bibi_xml.read()
        self.mock_storageClient_instance.get_file = fake_get

        fake_filepath = '/abc/cde'
        self.mock_storageClient_instance.clone_file = fake_filepath
        fake_world_sdf_string = '<sdf></sdf>'
        with patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.open', mock_open(read_data=fake_world_sdf_string), create=True) as m:
            # call target function
            world_sdf_string = SimulationResetStorage._get_sdf_world_from_storage(
                self.experiment_id,None)

        # assertions
        mock_get_header_token.assert_called()
        self.assertEqual(
            self.mock_storageClient_instance.clone_file, fake_filepath)

        self.assertNotEqual(world_sdf_string, fake_world_sdf_string)

    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.tempfile')
    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.get_all_neurons_as_dict')
    @patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.UserAuthentication.get_header_token')
    def test_get_brain_info_from_storage(self, mock_get_brain_info,
                                         mock_get_all_neurons_as_dict, mock_tempfile):

        dummy_dir = "/my/temp/dir"
        mock_tempfile.mkdtemp.return_value = dummy_dir

        self.mock_storageClient_instance.clone_file.return_value = os.path.join(
            dummy_dir, "brain.py")

        bibi_original_path = os.path.join(os.path.split(
            __file__)[0], "experiments", "experiment_data", "bibi_1.bibi")
        bibi_temp_path = os.path.join(tempfile.mkdtemp(), "bibi_test.xml")
        bibi_remote_path = os.path.join("/collab_dir", "bibi_test.xml")
        shutil.copyfile(bibi_original_path, bibi_temp_path)

        exp_temp_path = os.path.join(os.path.split(
            __file__)[0], "experiments", "experiment_data", "test_1.exc")

        def fake_get(*args, **kwargs):
            if args[2] == 'experiment_configuration.exc':
                with open(exp_temp_path) as exp_xml:
                    return exp_xml.read()
            with open(bibi_temp_path) as bibi_xml:
                return bibi_xml.read()
        self.mock_storageClient_instance.get_file = fake_get

        dummy_populations = {'pop1': slice(0, 1, 1), 'pop2': slice(1, 2, 1)}
        mock_get_all_neurons_as_dict.return_value = dummy_populations
        data_from_storage, populations, _ = SimulationResetStorage._get_brain_info_from_storage(
            self.experiment_id, None)
        self.assertEqual(data_from_storage,
                         os.path.join(dummy_dir, 'brain.py'))
        self.assertEqual(
            "[name: pop2\ntype: 1\nids: []\nstart: 1\nstop: 2\nstep: 1, name: pop1\ntype: 1\nids: []\nstart: 0\nstop: 1\nstep: 1]", str(populations))

    @patch("hbp_nrp_backend.rest_server.__SimulationResetStorage.SimulationResetStorage.resetFromStorageAll")
    def test_full_reset_ok(self, mock_reset_from_storage):
        with patch('hbp_nrp_backend.rest_server.__SimulationResetStorage.SimulationResetStorage._compute_payload') \
                as mock_compute_payload:
                    mock_reset_from_storage.side_effect = None
                    mock_compute_payload.return_value = '<sdf></sdf>', None, None
                    simulations[0].cle = mock.MagicMock()

                    response = self.client.put(self.correct_reset_url, data=json.dumps({
                        'resetType': ResetSimulationRequest.RESET_ROBOT_POSE,
                        'contextId' : None
                    }))
                    self.assertEqual(200, response.status_code)
                    simulations[0].cle.reset.side_effect = None
                    response = self.client.put(self.correct_reset_url, data=json.dumps({
                        'resetType': ResetSimulationRequest.RESET_FULL,
                        'contextId' : None
                    }))
                    simulations[0].cle.reset.assert_called()
                    self.assertEqual(response.status_code, 200)

if __name__ == '__main__':
    unittest.main()
