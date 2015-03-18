"""
Test to run the ExD configuration script
"""

__author__ = 'Lorenzo Vannucci'

from hbp_nrp_backend.exd_config.experiment_configuration_script \
    import generate_bibi, initialize_experiment
from hbp_nrp_cle.cle.ROSCLEClient import ROSCLEClient
import unittest
import os
import difflib
import rospy
import mock

rospy.ServiceProxy = mock.Mock(return_value=mock.Mock())

class TestExperimentConfigurationScript(unittest.TestCase):
    """
    Test the generation of the ExD config script
    """

    def test_initialize_experiment(self):
        directory = os.path.split(__file__)[0]
        experiment = os.path.join(directory, 'ExDXMLExample.xml')
        self.assertIsInstance(initialize_experiment(experiment, 'generate.xml'), ROSCLEClient)

    def test_generate_bibi(self):
        """
        Test the generation.
        """
        directory = os.path.split(__file__)[0]
        experiment = os.path.join(directory, 'ExDXMLExample.xml')
        generated_bibi = os.path.join(directory, 'generated_bibi.py')
        expected_generated_bibi = os.path.join(directory, 'expected_bibi.py')

        # Remove the generated file if it already exists.
        try:
            os.remove(generated_bibi)
        except OSError:
            pass

        # Generate bibi script file and compare it to an expected file.
        generate_bibi(experiment, generated_bibi)
        self.assertTrue(os.path.exists(generated_bibi))
        file1 = open(generated_bibi, 'r')
        file2 = open(expected_generated_bibi, 'r')
        diff = difflib.context_diff(file1.readlines(), file2.readlines())
        delta = ''.join(diff)
        self.maxDiff = None
        self.assertMultiLineEqual(delta, "")

if __name__ == '__main__':
    unittest.main()
