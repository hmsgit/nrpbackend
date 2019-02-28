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
Unit tests for user authentication
"""

__author__ = 'Oliver Denninger'

import unittest
from mock import patch, MagicMock
from flask import Flask
import flask

from hbp_nrp_backend.__UserAuthentication import UserAuthentication
from hbp_nrp_backend import NRPServicesWrongUserException

class FakeSimulation:
    def __init__(self, owner):
        self.ctx_id = None
        self.experiment_id = None
        self.owner = owner

class TestUserAuthentication(unittest.TestCase):

    def setUp(self):
        self.__app = Flask(__name__)

    def test_get_x_user_name_header(self):
        # ensure 'X-User-Name' header is used if available
        with self.__app.test_request_context('/test', headers={'X-User-Name': 'Test'}):
            owner = UserAuthentication.get_x_user_name_header()
        self.assertEqual(owner, 'Test')

        # ensure 'default-owner' is used if 'X-User-Name' header is not available
        with self.__app.test_request_context('/test'):
            owner = UserAuthentication.get_x_user_name_header()
        self.assertEqual(owner, 'default-owner')

    def test_get_header_token(self):
        # ensure 'Authorization' header is used if available
        with self.__app.test_request_context('/test', headers={'Authorization': 'Bearer aaa-bbb'}):
            token = UserAuthentication.get_header_token()
        self.assertEqual(token, 'aaa-bbb')

        # ensure 'no_token' is used if 'Authorization' header is not available
        with self.__app.test_request_context('/test'):
            token = UserAuthentication.get_header_token()
        self.assertEqual(token, 'no_token')


    def test_get_user(self):
        with self.__app.test_request_context('/test', headers={'X-User-Name': 'Test'}):
            self.assertEqual(UserAuthentication.get_user(), 'Test')
        with self.__app.test_request_context('/test', headers={}):
            self.assertEqual(UserAuthentication.get_user(), 'default-owner')
        with self.__app.test_request_context('/test', headers={'Authorization':'bearer my_incorrect_token'}):
            self.assertEqual(UserAuthentication.get_user(), 'default-owner')
        with patch("hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle.UserAuthentication.client") as client:
            client.get_user = MagicMock(return_value={'id': 'myid'})
            with self.__app.test_request_context('/test', headers={'Authorization':'bearer my_token'}):
                self.assertEqual(UserAuthentication.get_user(), 'myid')


    def test_can_modify(self):
        with self.__app.test_request_context('/test', headers={'X-User-Name': 'Test'}):
            self.assertTrue(UserAuthentication.can_modify(FakeSimulation('Test')))
            self.assertFalse(
                UserAuthentication.can_modify(FakeSimulation('default-owner')))

        with self.__app.test_request_context('/test'):
            self.assertFalse(UserAuthentication.can_modify(FakeSimulation('Test')))
            self.assertTrue(
                UserAuthentication.can_modify(FakeSimulation('default-owner')))

    def test_can_view(self):

        sim = FakeSimulation('Test')
        with self.__app.test_request_context('/test'):
            # UserAuthentication.NO_TOKEN should return False
            self.assertFalse(UserAuthentication.can_view(sim))

        with self.__app.test_request_context('/test', headers={'Authorization':'bearer my_token'}):
            with patch("hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle.UserAuthentication.client") as client:
                client.can_acess_experiment = MagicMock(return_value=True)
                # If failed to 'can_acess_experiment', it should default to false
                client.can_acess_experiment.side_effect = Exception('Test')
                self.assertFalse(UserAuthentication.can_view(sim))


if __name__ == '__main__':
    unittest.main()