"""
Test to run the ExD configuration script
"""

__author__ = 'Lorenzo Vannucci'

from hbp_nrp_backend.exd_config.experiment_configuration_script \
    import generate_bibi, initialize_experiment
from hbp_nrp_cle.cle.ROSCLEClient import ROSCLEClient
from hbp_nrp_cle.cle.ROSCLESimulationFactoryClient import ROSCLESimulationFactoryClient
import unittest
import os
import rospy
import mock

rospy.ServiceProxy = mock.Mock(return_value=mock.Mock())


class TestExperimentConfigurationScript(unittest.TestCase):
    """
    Test the generation of the ExD config script
    """

    def test_initialize_experiment(self):
        """
        Tests the experiment initialization.
        """

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

        # Remove the generated file if it already exists.
        if os.path.isfile(generated_bibi):
            os.remove(generated_bibi)

        # Generate bibi script file and compare it to an expected file.
        generate_bibi(experiment, generated_bibi, 'local')
        self.assertTrue(os.path.isfile(generated_bibi))

if __name__ == '__main__':
    unittest.main()
