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
Unit tests for the experiment clone rest call
"""
__author__ = 'Manos Angelidis'

import rospy
import os
import shutil
import tempfile
from mock import MagicMock, patch, ANY
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_commons.generated import exp_conf_api_gen


class TestExperimentWorldSDF(RestTest):

    def setUp(self):
        patch_StorageClient = patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient')
        self.addCleanup(patch_StorageClient.stop)
        self.mock_StorageClient = patch_StorageClient.start()
        self.mock_storageClient_instance = self.mock_StorageClient.return_value

    def test_experiment_clone_put_ok(self):
        experiment_id = 'fakeExpID'
        self.mock_storageClient_instance.clone_experiment_template_to_storage.return_value =  None
        response = self.client.put('/experiment/clone', data='{"exp_configuration_path": "fakePath"}')
        self.assertEqual(response.status_code, 200)


    def test_experiment_clone_put_not_ok(self):
        experiment_id = 'fakeExpID'
        response = self.client.put('/experiment/clone')
        self.assertEqual(response.status_code, 400)
