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
This module tests the sim config utility functions
"""

import unittest
from mock import Mock, patch
from hbp_nrp_commons.sim_config.SimConfUtil import SimConfUtil

__author__ = 'Hossain Mahmud'


class TestRobotManager(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_convertXSDPosetoPyPose(self):
        pose = None
        self.assertTrue(SimConfUtil.convertXSDPosetoPyPose(pose) is None)

        pose = Mock()
        self.mocked_pose = patch("hbp_nrp_cle.robotsim.RobotManager.Pose").start()
        self.mocked_pose.return_value.postion = "mocked_pose_position"
        self.mocked_transformation = patch("hbp_nrp_cle.robotsim.RobotManager.transformations").start()

        pose.ux.return_value = 1
        self.assertTrue(SimConfUtil.convertXSDPosetoPyPose(pose), self.mocked_pose.return_value)

        pose.ux = None
        self.assertTrue(SimConfUtil.convertXSDPosetoPyPose(pose), self.mocked_pose.return_value)

        self.mocked_transformation.quaternion_from_euler.return_value = [None, None, None, None]
        self.assertTrue(SimConfUtil.convertXSDPosetoPyPose(pose), self.mocked_pose.return_value)

        self.mocked_pose.stop()
        self.mocked_transformation.stop()


if __name__ == '__main__':
    unittest.main()
