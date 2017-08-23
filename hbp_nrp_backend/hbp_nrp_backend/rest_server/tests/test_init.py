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
Code for testing all classes in hbp_nrp_backend.rest_server.__init__
"""

import unittest
from mock import Mock, patch
from hbp_nrp_backend.rest_server import NRPServicesGeneralException, NRPServicesStateException
from hbp_nrp_backend.rest_server import NRPServicesDatabaseTimeoutException, NRPServicesDuplicateNameException
from hbp_nrp_backend.rest_server import NRPServicesDatabaseException, db_create_and_check
from hbp_nrp_backend.rest_server.tests.test_config import LocalConfigTest
from flask import Flask
from flask_sqlalchemy import SQLAlchemy


class TestInit(unittest.TestCase):
    """
    This class tests all classes in hbp_nrp_backend.rest_server.__init__
    """

    def test_nrp_services_general_exception(self):
        """
        This method tests the class hbp_nrp_backend.rest_server.NRPServicesGeneralException
        """
        nsge = NRPServicesGeneralException("StringA", "StringB")
        self.assertEqual(nsge.__str__(), "'StringA' (StringB)")

    def test_nrp_services_state_exception(self):
        """
        This method tests the class hbp_nrp_backend.rest_server.NRPServicesStateException
        """
        nsge = NRPServicesStateException("StringA")
        self.assertEqual(nsge.__str__(), "'StringA' (Transition error)")

    def test_nrp_services_db_exception(self):
        """
        This method tests the class hbp_nrp_backend.rest_server.NRPServicesDatabaseException
        """
        nsge = NRPServicesDatabaseException("StringA")
        self.assertEqual(nsge.__str__(), "'StringA' (Database error)")

    def test_nrp_services_db_timeout_exception(self):
        """
        This method tests the class hbp_nrp_backend.rest_server.NRPServicesDatabaseTimeoutException
        """
        nsge = NRPServicesDatabaseTimeoutException()
        self.assertEqual(nsge.__str__(), "'Database connection timeout' (Database error)")

    def test_nrp_services_duplicate_name_exception(self):
        """
        This method tests the class hbp_nrp_backend.rest_server.NRPServicesDuplicateNameException
        """
        nsge = NRPServicesDuplicateNameException("StringA")
        self.assertEqual(nsge.__str__(), "'StringA' (Duplicate name error)")

    @patch('multiprocessing.Process')
    def test_db_create_and_check_local(self, mock_process):
        """
        This method tests the function test_db_create_and_check for
        the local database configuration
        """
        mock_process.is_alive = Mock(return_value=True)

        app = Flask("test_app")
        app.config.from_object(LocalConfigTest)
        db = SQLAlchemy(app)
        try:
            self.assertRaises(NRPServicesDatabaseTimeoutException, db_create_and_check, db)
            mock_process.start.assert_called_once()
            mock_process.join.assert_called_once()
            mock_process.is_alive.assert_called_once()
            mock_process.terminate.assert_called_once()
        except Exception as e:
            self.fail("db_create_and_check raised an exception in local mode: " + str(e))


if __name__ == '__main__':
    unittest.main()
