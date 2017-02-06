"""
This module contains the unit tests for the cle launcher shutdown
"""

import unittest
import os
from mock import patch, Mock
from hbp_nrp_cleserver.server import CLELauncher
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen


@patch("hbp_nrp_cleserver.server.CLELauncher.os", new=Mock())
class TestCLELauncherShutdown(unittest.TestCase):
    def setUp(self):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "BIBI/milestone2.xml")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "ExDConf/ExDXMLExample.xml")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        self.launcher = CLELauncher.CLELauncher(exd, bibi, "/somewhere/over/the/rainbow", "gz_host", None, 42)
        self.launcher.cle_server = Mock()
        self.launcher.gzweb = Mock()
        self.launcher.gzserver = Mock()

    def test_shutdown_stops_everything(self):
        self.launcher.shutdown()
        self.__assert_everything_properly_shut_down()

    def __assert_everything_properly_shut_down(self):
        self.launcher.gzweb.stop.assert_called_once_with()
        self.launcher.gzserver.stop.assert_called_once_with()
        self.launcher.cle_server.shutdown.assert_called_once_with()

    def test_notify_throws_but_assets_shut_down(self):
        self.launcher.cle_server.notify_start_task.side_effect = Exception
        self.launcher.shutdown()
        self.__assert_everything_properly_shut_down()

    def test_gzserver_shut_down_if_gzweb_throws(self):
        self.launcher.gzweb.stop.side_effect = Exception
        self.launcher.shutdown()
        self.__assert_everything_properly_shut_down()

    def test_cleserver_shutdown_raises(self):
        self.launcher.cle_server.shutdown.side_effect = Exception
        self.launcher.shutdown()
        self.__assert_everything_properly_shut_down()
