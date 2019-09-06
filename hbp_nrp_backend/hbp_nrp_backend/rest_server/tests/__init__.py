"""
This module contains unit tests for the experiment backend
"""

__author__ = 'GeorgHinkel'

import unittest
from hbp_nrp_backend.rest_server import app


class RestTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        app.before_first_request_funcs = []
        self.client = app.test_client()

    @classmethod
    def tearDownClass(self):
        pass
