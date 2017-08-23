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
This module contains the unit tests for the cle launcher initialization
"""

import unittest
import os
from mock import patch, MagicMock, Mock
from hbp_nrp_cleserver.server import CLELauncher
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from hbp_nrp_cleserver.server.LuganoVizClusterGazebo import XvfbXvnError
from hbp_nrp_cle.mocks.robotsim import MockRobotControlAdapter, MockRobotCommunicationAdapter
from threading import Thread, Event

class MockedGazeboHelper(object):

    def load_gazebo_world_file(self, world):
        return {}, {}

    def __getattr__(self, x):
        return Mock()

class MockedClosedLoopEngine(Mock):

    DEFAULT_TIMESTEP = 0.02


MockOs = Mock()
MockOs.environ = {'NRP_MODELS_DIRECTORY': '/somewhere/near/the/rainbow',
                  'ROS_MASTER_URI': "localhost:0815"}
MockOs.path.join.return_value = "/a/really/nice/place"

class SomeWeiredTFException(Exception):
    pass

@patch("hbp_nrp_cleserver.server.CLELauncher.RosControlAdapter", new=MockRobotControlAdapter)
@patch("hbp_nrp_cleserver.server.CLELauncher.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
@patch("hbp_nrp_cleserver.server.CLELauncher.LocalGazeboBridgeInstance", new=Mock())
@patch("hbp_nrp_cleserver.server.CLELauncher.GazeboHelper", new=MockedGazeboHelper)
@patch("hbp_nrp_cle.cle.ClosedLoopEngine.GazeboHelper", new=MockedGazeboHelper)
@patch("hbp_nrp_cleserver.server.CLELauncher.ClosedLoopEngine", new=MockedClosedLoopEngine())
@patch("hbp_nrp_cleserver.server.CLELauncher.instantiate_communication_adapter", new=Mock())
@patch("hbp_nrp_cleserver.server.CLELauncher.instantiate_control_adapter", new=Mock())
@patch("hbp_nrp_cleserver.server.CLELauncher.os", new=MockOs)
@patch("hbp_nrp_cleserver.server.CLELauncher.CLELauncher._get_robot_abs_path", new=Mock(return_value="/a/robot/under/the/rainbow/model.sdf"))
@patch("hbp_nrp_cleserver.server.CLELauncher.ROSNotificator", new=Mock())
class TestCLELauncherInit(unittest.TestCase):
    def setUp(self):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "experiment_data/milestone2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "experiment_data/ExDXMLExample.exc")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        with patch("hbp_nrp_cleserver.server.CLELauncher.os", MockOs):
            self.launcher = CLELauncher.CLELauncher(exd, bibi, "/somewhere/over/the/rainbow", "local", None, 42, None)

    def test_gazebo_location_not_supported_throws_exception(self):
        self.launcher._CLELauncher__gzserver_host = 'not_supported'
        with self.assertRaises(Exception):
            self.launcher.cle_function_init("world_file")

    @patch("hbp_nrp_cleserver.server.CLELauncher.ROSCLEServer")
    @patch("hbp_nrp_cleserver.server.CLELauncher.LocalGazeboServerInstance")
    def test_gazebo_start_exception_catches_xvfbxvn_error(self, mock_gazebo, mock_server):
        mock_gazebo().start.side_effect = XvfbXvnError
        exception_caught = False
        try:
            self.launcher.cle_function_init("world_file")
        except Exception, e:
            self.assertTrue(str(e).startswith("Recoverable error occurred"), msg=e)
            exception_caught = True
        self.assertTrue(exception_caught)

    @patch("hbp_nrp_cleserver.server.CLELauncher.nrp")
    @patch("hbp_nrp_cleserver.server.CLELauncher.ROSCLEServer")
    @patch("hbp_nrp_cleserver.server.CLELauncher.LocalGazeboServerInstance")
    def test_wrong_transfer_function_aborts_initialization(self, mock_gazebo, mock_server, mock_nrp):
        mock_nrp.set_transfer_function.side_effect = SomeWeiredTFException
        with self.assertRaises(SomeWeiredTFException):
            self.launcher.cle_function_init("world_file")

    def __launch_callback(self, *_):
        self.launcher.gzserver.gazebo_died_callback()

    @patch("hbp_nrp_cleserver.server.CLELauncher.nrp")
    @patch("hbp_nrp_cleserver.server.CLELauncher.ROSCLEServer")
    @patch("hbp_nrp_cleserver.server.CLELauncher.LocalGazeboServerInstance")
    def test_gazebo_crash_aborts_initialization(self, mock_gazebo, mock_server, mock_nrp):
        mock_gazebo().start.side_effect = self.__launch_callback
        exception_caught = False
        try:
            self.launcher.cle_function_init("world_file")
        except Exception, e:
            self.assertTrue(str(e).startswith("The simulation must abort"), msg=e)
            exception_caught = True
        self.assertTrue(exception_caught)
        mock_server().lifecycle.failed.assert_called_once_with()
        mock_server().lifecycle.stopped.assert_called_once_with()

    @patch("hbp_nrp_cleserver.server.CLELauncher.nrp")
    @patch("hbp_nrp_cleserver.server.CLELauncher.PlaybackServer")
    @patch("hbp_nrp_cleserver.server.CLELauncher.LocalGazeboServerInstance")
    def test_gazebo_crash_aborts_playback_initialization(self, mock_gazebo, mock_server, mock_nrp):
        mock_gazebo().start.side_effect = self.__launch_callback
        exception_caught = False
        try:
            self.launcher._CLELauncher__playback_path = 'foo'
            self.launcher.cle_function_init("world_file")
        except Exception, e:
            self.assertTrue(str(e).startswith("The simulation must abort"), msg=e)
            exception_caught = True
        finally:
            self.launcher._CLELauncher__playback_path = None
        self.assertTrue(exception_caught)
        mock_server().lifecycle.failed.assert_called_once_with()
        mock_server().lifecycle.stopped.assert_called_once_with()
