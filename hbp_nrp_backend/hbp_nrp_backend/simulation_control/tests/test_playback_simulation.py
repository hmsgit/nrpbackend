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

from mock import Mock, patch, MagicMock, ANY
from hbp_nrp_commons.MockUtil import MockUtil
import unittest
import os

from hbp_nrp_backend.simulation_control.__PlaybackSimulationLifecycle import PlaybackSimulationLifecycle

PATH = os.path.split(__file__)[0]

__author__ = 'Hossain Mahmud'

_base_ = "hbp_nrp_backend.simulation_control.__PlaybackSimulationLifecycle."


@patch(_base_ + 'UserAuthentication', new=MagicMock())
class TestPlaybackSimulationLifecycle(unittest.TestCase):
    def setUp(self):

        # Mock all external imported modules
        self.m_os = MockUtil.fakeit(self, _base_ + 'os')
        self.m_storage = MockUtil.fakeit(self, _base_ + 'StorageClient')
        self.m_ziputil = MockUtil.fakeit(self, _base_ + 'ZipUtil')

        # Prepare dummy data for the instance
        self.simulation = Mock()
        self.simulation.sim_id = 42
        self.simulation.experiment_conf = "ExDXMLExample.xml"
        self.simulation.experiment_id = 'my_awesome_exp'
        self.simulation.state_machines = []
        self.simulation.playback_path = "a/path/to/some.zip"
        self.simulation.private = None

        # Mock the base cass of PlaybackSimulationLifecycle
        self.m_base = MockUtil.fake_base(self, PlaybackSimulationLifecycle)
        self.m_base.simulation.return_value = self.simulation

        self.playback_lifecycle = PlaybackSimulationLifecycle(self.simulation)

    def tearDown(self):
        pass

    def test_init_playback(self):
        self.m_base.__init__.assert_callled_once_with(self.simulation, 'created')

    def test_initialize_playback(self):
        self.playback_lifecycle.prepare_record_for_playback = Mock()
        self.playback_lifecycle.initialize('some state')

        self.playback_lifecycle.prepare_record_for_playback.assert_called_once()
        self.m_base.initialize.assert_callled_once_with('some state')

    def test_start_playback(self):
        self.playback_lifecycle.start('some state')
        self.m_base.start.assert_not_called()

    def test_stop_playback(self):
        self.playback_lifecycle.stop('some state')
        self.m_base.stop.assert_callled_once_with(self.simulation, 'some state')

    def test_prepare_record_for_playback(self):
        self.m_storage.return_value.get_simulation_directory.return_value = '/simulation/dir'
        self.m_os.path.exists.return_value = False

        from mock import mock_open
        with patch("__builtin__.open", mock_open()):
            self.playback_lifecycle.prepare_record_for_playback()

        self.m_storage.return_value.get_file.assert_callled_once_with(
            ANY, 'my_awesome_exp%2Frecordings', 'some.zip', by_name=True, zipped=True)

        self.m_os.makedirs.assert_callled_once_with('simulation/dir/a/path/to')
        self.m_ziputil.extractall.assert_callled_once_with('simulation/dir/a/path/to/some.zip',
                                                           'simulation/dir/a/path/to', True)
