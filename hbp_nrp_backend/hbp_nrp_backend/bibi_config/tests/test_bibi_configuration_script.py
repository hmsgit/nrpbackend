"""
Test to run the bibi configuration script
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.bibi_config.bibi_configuration_script import generate_cle
import unittest
import os


class TestScript(unittest.TestCase):

    def test_script(self):
        # for testing purposes, delete before check in
        directory = os.path.split(__file__)[0]
        milestone2 = os.path.join(directory, 'milestone2.xml')
        generate_cle(milestone2, os.path.join(directory, 'generated_cle_script.py'))


if __name__ == '__main__':
    unittest.main()
