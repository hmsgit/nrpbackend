"""
This class tests the main startup script.
"""
__author__ = 'StefanDeser'

from argparse import Namespace
from hbp_nrp_backend import runserver
from mock import MagicMock
from testfixtures import log_capture
import unittest

class TestScript(unittest.TestCase):
    def setUp(self):
        # a simple mock for the application server
        class MockApp: pass
        self.app = MockApp()
        self.app.run = MagicMock()

        self.args = Namespace()
        self.args.logfile = None
        self.args.port = None

    @log_capture()
    def test_run_server_no_arguments(self, logcapture):
        runserver.run_server(self.app, self.args)
        logcapture.check(
                         ('hbp_nrp_backend.runserver', 'WARNING', 'Could not write to specified logfile or no logfile specified, logging to stdout now!'),
                         ('hbp_nrp_backend.runserver', 'WARNING', 'Could not parse port, will use default port: ' + str(runserver.DEFAULT_PORT)),
                         ('hbp_nrp_backend.runserver', 'INFO', 'Starting the REST backend server now ...'),
                         ('hbp_nrp_backend.runserver', 'INFO', 'REST backend server terminated.')
                         )
        self.assertTrue(self.app.run.called)
        self.app.run.assert_called_with(port=runserver.DEFAULT_PORT, host=runserver.DEFAULT_HOST)

if __name__ == '__main__':
    unittest.main()
