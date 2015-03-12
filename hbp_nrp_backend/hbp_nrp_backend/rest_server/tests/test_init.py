"""
Code for testing all classes in hbp_nrp_backend.rest_server.__init__
"""

import unittest
from hbp_nrp_backend.rest_server import NRPServicesGeneralException


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

if __name__ == '__main__':
    unittest.main()