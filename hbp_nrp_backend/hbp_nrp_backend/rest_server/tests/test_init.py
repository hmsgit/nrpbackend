"""
Code for testing all classes in hbp_nrp_backend.rest_server.__init__
"""

import unittest
from hbp_nrp_backend.rest_server import NRPServicesGeneralException, NRPServicesStateException
from hbp_nrp_backend.rest_server import NRPServicesDatabaseTimeoutException
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

    def test_db_create_and_check_local(self):
        """
        This method tests the function test_db_create_and_check for
        the local database configuration
        """
        app = Flask("test_app")
        app.config.from_object(LocalConfigTest)
        db = SQLAlchemy(app)
        try:
            db_create_and_check(db)
        except Exception as e:
            self.fail("db_create_and_check raised an exception in local mode: " + str(e))


if __name__ == '__main__':
    unittest.main()
