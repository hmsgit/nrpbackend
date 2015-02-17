"""
Unit tests for user authentication
"""

__author__ = 'Oliver Denninger'

import unittest
from flask import Flask
import flask

from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication


class TestUserAuthentication(unittest.TestCase):

    def setUp(self):
        self.__app = Flask(__name__)

    def test_get_x_user_name_header(self):
        # ensure 'X-User-Name' header is used if available
        with self.__app.test_request_context('/test', headers={'X-User-Name': 'Test'}):
            owner = UserAuthentication.get_x_user_name_header(flask.request)
        self.assertEqual(owner, 'Test')

        # ensure 'default-owner' is used if 'X-User-Name' header is not available
        with self.__app.test_request_context('/test'):
            owner = UserAuthentication.get_x_user_name_header(flask.request)
        self.assertEqual(owner, 'default-owner')

    def test_matches_x_user_name_header(self):
        with self.__app.test_request_context('/test', headers={'X-User-Name': 'Test'}):
            self.assertTrue(UserAuthentication.matches_x_user_name_header(flask.request, 'Test'))
            self.assertFalse(
                UserAuthentication.matches_x_user_name_header(flask.request, 'default-owner'))

        with self.__app.test_request_context('/test'):
            self.assertFalse(UserAuthentication.matches_x_user_name_header(flask.request, 'Test'))
            self.assertTrue(
                UserAuthentication.matches_x_user_name_header(flask.request, 'default-owner'))
