"""
This module contains unit tests for the experiment backend
"""

__author__ = 'GeorgHinkel'

import unittest
import mock
from hbp_nrp_backend.rest_server import init, app


class RestTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.mock_environment = mock.patch.dict('os.environ', {'APP_SETTINGS': 'config.TestConfig'})
        self.mock_environment.start()

        init()
        self.client = app.test_client()

    @classmethod
    def tearDownClass(self):
        self.mock_environment.stop()
