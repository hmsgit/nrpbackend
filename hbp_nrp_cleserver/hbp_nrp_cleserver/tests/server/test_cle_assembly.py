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
import os
from mock import patch
from hbp_nrp_cleserver.server.ServerConfigurations import SynchronousNestSimulation, SynchronousRobotRosNest
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from hbp_nrp_cle.mocks.robotsim import MockRobotControlAdapter, MockRobotCommunicationAdapter
from hbp_nrp_backend import NRPServicesGeneralException

PATH = os.path.split(__file__)[0]


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
        _dir = os.path.split(__file__)[0]
        with open(os.path.join(_dir, "experiment_data/milestone2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(_dir, "experiment_data/ExDXMLExample.exc")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        exd.path = "/somewhere/over/the/rainbow/exc"
        exd.dir = "/somewhere/over/the/rainbow"
        bibi.path = "/somewhere/over/the/rainbow/bibi"
        with patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.StorageClient"):
            self.launcher = SynchronousNestSimulation(42, exd, bibi, gzserver_host="local")

    def tearDown(self):
        pass

    def test_custom_brain_fails(self):
        self.launcher._storageClient.get_custom_models.return_value = [
            {'path': "brains/brain_.zip"}    # DON'T PUT brain.zip HERE. FIX _extract_brain_zip impl
        ]
        self.launcher._storageClient.get_custom_model.return_value = r'awesome brain data'
        self.launcher._simDir = os.path.join(PATH, 'experiment_data')
        with open(os.path.join(PATH, "experiment_data/milestone2_2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        self.launcher._SimulationAssembly__bibi = bibi
        rng_seed = 0
        self.assertRaises(NRPServicesGeneralException, self.launcher._load_brain, rng_seed)

    @patch("zipfile.ZipFile")
    def test_custom_brain_succeeds(self, mocked_zip):
        self.launcher._storageClient.get_custom_models.return_value = [{'path':"brains/brain.zip"}]
        self.launcher._storageClient.get_custom_model.return_value = r'awesome brain data'
        self.launcher._simDir = os.path.join(PATH, 'experiment_data')
        with open(os.path.join(PATH, "experiment_data/milestone2_2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        self.launcher._SimulationAssembly__bibi = bibi

        self.launcher._extract_brain_zip()
        mocked_zip.assert_called_once_with(os.path.join(PATH, 'experiment_data', 'brain.zip'))

    def test_load_brain_storage(self):
        self.launcher._storageClient.get_folder_uuid_by_name.return_value = 'brains'
        self.launcher._simDir = PATH
        with open(os.path.join(PATH, 'experiment_data/braitenberg.py')) as brain:
            brain_contents = brain.read()
        self.launcher._storageClient.get_file.return_value = brain_contents
        with open(os.path.join(PATH, "experiment_data/milestone2_1.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        self.launcher._SimulationAssembly__bibi = bibi
        braincontrol, braincomm, brainfilepath, neurons_config = self.launcher._load_brain(1234)
        self.assertEqual(brainfilepath, os.path.join(PATH, 'braitenberg.py'))
        self.assertEqual(neurons_config, {u'sensors': slice(
            0L, 5L, None), u'actors': slice(5L, 8L, None)})

    def test_invalid_simulation(self):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "experiment_data/milestone2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "experiment_data/ExDXMLExample.exc")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        exd.path = "/somewhere/over/the/rainbow/exc"
        exd.dir = "/somewhere/over/the/rainbow"
        exd.physicsEngine = None
        bibi.path = "/somewhere/over/the/rainbow/bibi"

        try:
            SynchronousNestSimulation(42, exd, bibi, gzserver_host="local")
        except Exception:
            self.fail("FAILED test_cle_assembly.test_invalid_simulation(). Should have initialized")

        self.assertRaises(Exception, SynchronousNestSimulation, 42, exd, bibi, gzserver_host="bullshit")

    @patch("hbp_nrp_cle.robotsim.RosControlAdapter.RosControlAdapter", new=MockRobotControlAdapter)
    @patch("hbp_nrp_cle.robotsim.RosCommunicationAdapter.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
    def test_load_robot_adapters(self):
        (comm, ctrl) = self.launcher._create_robot_adapters()
        self.assertIsNotNone(comm)
        self.assertIsNotNone(ctrl)


class TestCLEGazeboSimulationAssemblyRobot(TestCLEGazeboSimulationAssembly):

    def setUp(self):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "experiment_data/milestone2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "experiment_data/ExDXMLExample.exc")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        exd.path = "/somewhere/over/the/rainbow/exc"
        exd.dir = "/somewhere/over/the/rainbow"
        bibi.path = "/somewhere/over/the/rainbow/bibi"

        with patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.StorageClient"):
            self.launcher = SynchronousRobotRosNest(42, exd, bibi, gzserver_host="local")

    @patch("hbp_nrp_cle.robotsim.RosRobotControlAdapter.RosRobotControlAdapter", new=MockRobotControlAdapter)
    @patch("hbp_nrp_cle.robotsim.RosCommunicationAdapter.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
    def test_load_robot_adapters(self):
        (comm, ctrl) = self.launcher._create_robot_adapters()
        self.assertIsNotNone(comm)
        self.assertIsNotNone(ctrl)


if __name__ == '__main__':
    unittest.main()