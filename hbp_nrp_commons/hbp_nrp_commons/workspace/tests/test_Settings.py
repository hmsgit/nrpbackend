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
Unit tests for the sim config model
"""

import unittest
import os
from mock import patch, MagicMock
from hbp_nrp_commons.MockUtil import MockUtil

__author__ = 'Hossain Mahmud'

_base_path = 'hbp_nrp_commons.workspace.Settings.'


@patch(_base_path + 'logging', new=MagicMock())
class TestSimConfig(unittest.TestCase):
    def setUp(self):
        # mock os
        self.m_os = MockUtil.fakeit(self, _base_path + 'os')
        # restore os.path.join and os.path.dirname
        self.m_os.path.join = os.path.join
        self.m_os.path.dirname = os.path.dirname
        self.m_os.environ = {
            'HOME': '/home/dir',
            'HBP': '/hbp/dir',
            'NRP_SIMULATION_DIR': '/sim/dir',
            'NRP_MODELS_DIRECTORY': '/models/dir'
        }

    def tearDown(self):
        pass

    def test_init_Settings(self):
        # NOTE: top level import won't work here as os is NOT MOCKED at the point
        # Clear the Singleton instance (if exists), and force a new copy
        from hbp_nrp_commons.workspace.Settings import _Settings
        _Settings._Settings__instance = None

        settings = _Settings()
        self.assertEqual(settings.nrp_home, '/hbp/dir')
        self.assertEqual(settings.sim_dir_symlink, '/sim/dir')
        self.assertEqual(settings.nrp_models_directory, '/models/dir')


if __name__ == '__main__':
    unittest.main()
