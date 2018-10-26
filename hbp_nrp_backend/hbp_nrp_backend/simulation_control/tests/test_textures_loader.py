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
This module tests the backend implementation of the simulation lifecycle
"""

from mock import Mock, patch
import unittest
import rospy
from hbp_nrp_backend.simulation_control.__TexturesLoader import TexturesLoader

__author__ = 'Manos Angelidis'


class TestBackendSimulationLifecycle(unittest.TestCase):

    @patch("rospy.ServiceProxy")
    def test_textures_load_ok(self, mock_service_proxy):
        with patch("hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.get_textures", return_value=[{"name": "test.png"}]):
            loader = TexturesLoader()
            loader.load_textures('fakeToken', 'fakeExperiment')
            mock_service_proxy.assert_called_once() 

    @patch("rospy.ServiceProxy", side_effect=rospy.ROSException())
    def test_textures_load_not_ok(self, mock_service_proxy):
        with patch("hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.get_textures", return_value=[{"name": "test.png"}]):
            loader = TexturesLoader()
            loader.load_textures('fakeToken','fakeExperiment')
            mock_service_proxy.assert_called_once()