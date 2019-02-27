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
This module contains the unit tests for the cle launcher
"""

import unittest
from mock import patch, MagicMock, Mock, mock_open, ANY
from hbp_nrp_cle.robotsim.RobotManager import Robot
from hbp_nrp_cleserver.server.ServerConfigurations import SynchronousNestSimulation, SynchronousRobotRosNest
from hbp_nrp_cle.mocks.robotsim import MockRobotControlAdapter, MockRobotCommunicationAdapter
from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_commons.MockUtil import MockUtil

def robot_value():
    return "robots/this_is_a_robot.sdf"


class CustomModel(object):
    def __init__(self):
        self.customModelPath = None  # only used for custom brains, remove when refactored
        self.assetPath = None
        self.value = robot_value
        self.path = 'test'
        self.robotId = 'robot'


class TestCLEGazeboSimulationAssembly(unittest.TestCase):
    def setUp(self):
        self.m_ziputil = MockUtil.fakeit(self, 'hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.ZipUtil')

        self.m_simconf = MagicMock()
        self.m_simconf.sim_id = 123
        self.m_simconf.token = 123
        self.m_simconf.experiment_id = 'experiment'
        self.m_simconf.timeout = 'YYYY-MM-DD-HH:MM:SS'
        self.m_simconf.timeout_type = 'real'
        self.m_simconf.sim_dir = '/my/experiment'
        self.m_simconf.gzserver_host = 'local'
        self.m_simconf.ros_notificator = 'real'
        self.m_simconf.world_model.resource_path.abs_path = '/my/experiment/world.sdf'
        self.m_simconf.brain_model.is_custom = False
        self.m_simconf.brain_model.resource_path.rel_path = 'brain.sdf'
        self.m_simconf.brain_model.resource_path.abs_path = '/my/experiment/brain.sdf'
        # alternate set is_custom in individual test
        self.m_simconf.brain_model.zip_path.rel_path = 'brains/brain.zip'
        self.m_simconf.brain_model.zip_path.abs_path = '/my/experiment/brains/brain.sdf'
        self.m_simconf.robot_models = {
            'bb8': Robot('bb8', '/my/experiment/bb8/model.sdf', 'my bb8', Mock()),
            'r2d2': Robot('r2d2', '/my/experiment/bb8/model.sdf', 'my r2d2', Mock(),
                          True, '/my/experiment/ros.launch')
        }
        self.m_simconf.retina_config = 'retina_configuration'
        self.m_simconf.ext_robot_controller = 'xyz.model'

        with patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.StorageClient"):
            self.launcher = SynchronousNestSimulation(self.m_simconf)

    def tearDown(self):
        pass

    def test_custom_brain_fails(self):
        self.m_simconf.brain_model.is_custom = True

        self.launcher._storageClient.get_custom_models.return_value = [{'path': "brains/brain_.zip"}]
        self.launcher._storageClient.get_custom_model.return_value = r'awesome brain data'

        self.assertRaises(NRPServicesGeneralException, self.launcher._load_brain)

    def test_custom_brain_succeeds(self):
        self.launcher._storageClient.get_custom_models.return_value = [{'path': "brains/brain.zip"}]
        self.launcher._storageClient.get_custom_model.return_value = r'awesome brain data'

        self.m_simconf.brain_model.zip_path.rel_path = 'brains/brain.zip'
        self.m_simconf.brain_model.zip_path.abs_path = '/my/experiment/brains/brain.zip'

        with patch("__builtin__.open", mock_open(read_data='bibi')):
            self.launcher._extract_brain_zip()

        self.m_ziputil.extractall.assert_called_once_with(
            overwrite=False, flatten=True, zip_abs_path='/my/experiment/brains/brain.zip',
            extract_to='/my/experiment'
        )

    def test_invalid_simulation(self):
        self.m_simconf.physics_engine = None
        try:
            SynchronousNestSimulation(self.m_simconf)
        except Exception:
            self.fail("FAILED test_cle_assembly.test_invalid_simulation(). Should have initialized")

        self.m_simconf.gzserver_host = 'something not supported'
        self.assertRaises(Exception, SynchronousNestSimulation, self.m_simconf)

    @patch("hbp_nrp_cle.robotsim.RosControlAdapter.RosControlAdapter", new=MockRobotControlAdapter)
    @patch("hbp_nrp_cle.robotsim.RosCommunicationAdapter.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
    def test_load_robot_adapters(self):
        (comm, ctrl) = self.launcher._create_robot_adapters()
        self.assertIsNotNone(comm)
        self.assertIsNotNone(ctrl)


class TestCLEGazeboSimulationAssemblyRobot(TestCLEGazeboSimulationAssembly):

    def setUp(self):
        super(TestCLEGazeboSimulationAssemblyRobot, self).setUp()

        # override launcher
        with patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.StorageClient"):
            self.launcher = SynchronousRobotRosNest(self.m_simconf)

    @patch("hbp_nrp_cle.robotsim.RosRobotControlAdapter.RosRobotControlAdapter", new=MockRobotControlAdapter)
    @patch("hbp_nrp_cle.robotsim.RosCommunicationAdapter.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
    def test_load_robot_adapters(self):
        (comm, ctrl) = self.launcher._create_robot_adapters()
        self.assertIsNotNone(comm)
        self.assertIsNotNone(ctrl)


if __name__ == '__main__':
    unittest.main()