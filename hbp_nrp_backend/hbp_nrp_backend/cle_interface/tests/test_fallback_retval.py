"""
Tests the fallback-retval decorator
"""

from hbp_nrp_backend.cle_interface.ROSCLEClient import fallback_retval, ROSCLEClientException
import unittest


__author__ = "Georg Hinkel"

class TestFallbackRetval(unittest.TestCase):

    @fallback_retval(42)
    def fallback_test(self, n):
        if n == 1:
            raise ROSCLEClientException()
        else:
            return 1

    def test_fallback_retval(self):
        self.assertEqual(1, self.fallback_test(42))
        self.assertEqual(42, self.fallback_test(1))
