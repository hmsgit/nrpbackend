"""
This module contains the unit tests for the cle launcher
"""

import unittest
import os
from mock import patch, Mock
from hbp_nrp_cleserver.server import CLELauncher
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen

class TestCLELauncher(unittest.TestCase):
    def setUp(self):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "BIBI/milestone2.xml")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "ExDConf/ExDXMLExample.xml")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        self.launcher = CLELauncher.CLELauncher(exd, bibi, "/somewhere/over/the/rainbow", "gz_host", None, 42)

    def test_robot_path_sdf(self):
        robot_file = self.launcher._get_robot_abs_path("robots/this_is_a_robot.sdf")
        self.assertEqual(robot_file, "/somewhere/over/the/rainbow/robots/this_is_a_robot.sdf")
        self.assertIsNone(self.launcher._CLELauncher__tmp_robot_dir)

    @patch("tempfile.mkdtemp")
    @patch("zipfile.ZipFile")
    def test_robot_path_zip(self, mocked_zip, mocked_temp):
        mocked_temp.return_value = "/tmp/under/the/rainbow"
        robot_file = self.launcher._get_robot_abs_path("robots/this_is_a_robot.zip")

        self.assertEqual(self.launcher._CLELauncher__tmp_robot_dir, "/tmp/under/the/rainbow")
        self.assertEqual(robot_file, "/tmp/under/the/rainbow/this_is_a_robot/model.sdf")

        self.assertTrue(mocked_temp.called)
        mocked_zip.assert_called_once_with("/somewhere/over/the/rainbow/robots/this_is_a_robot.zip")
        mocked_zip().__enter__().getinfo.assert_called_once_with("this_is_a_robot/model.sdf")
        mocked_zip().__enter__().extractall.assert_called_once_with(path="/tmp/under/the/rainbow")
