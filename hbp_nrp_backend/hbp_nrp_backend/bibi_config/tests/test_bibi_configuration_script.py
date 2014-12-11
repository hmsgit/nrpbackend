"""
Test to run the bibi configuration script
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.bibi_config.bibi_configuration_script import generate_cle
import unittest
import difflib
import os


class TestScript(unittest.TestCase):

    def test_script(self):
        # for testing purposes, delete before check in
        directory = os.path.split(__file__)[0]
        milestone2 = os.path.join(directory, 'milestone2.xml')
        generated_script = os.path.join(directory, 'generated_cle_script.py')
        milestone2_script = os.path.join(directory, 'milestone2_cle_script.py')
        generate_cle(milestone2, generated_script)

        file1 = open(generated_script, 'r')
        file2 = open(milestone2_script, 'r')
        diff = difflib.context_diff(file1.readlines(), file2.readlines())
        delta = ''.join(diff)
        self.maxDiff = None
        self.assertMultiLineEqual(delta, "")


if __name__ == '__main__':
    unittest.main()
