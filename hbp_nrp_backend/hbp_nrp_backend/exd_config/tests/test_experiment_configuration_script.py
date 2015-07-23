"""
Test to run the ExD configuration script
"""

__author__ = 'Lorenzo Vannucci'

from hbp_nrp_backend.exd_config.experiment_configuration_script \
    import generate_bibi, generate_experiment_control, initialize_experiment
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClient
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
        self.assertIsInstance(initialize_experiment(experiment, 'generate.xml', 0), ROSCLEClient)

    def test_generate_bibi(self):
        """
        Test the generation.
        """

        directory = os.path.split(__file__)[0]
        experiment = os.path.join(directory, 'ExDXMLExample.xml')
        generated_bibi = os.path.join(directory, 'generated_bibi.py')

        with mock.patch('hbp_nrp_backend.exd_config.experiment_configuration_script'
                        '.generated_experiment_api') as p:
            with mock.patch('hbp_nrp_backend.exd_config.experiment_configuration_script'
                            '.bibi_configuration_script') as pp:
                p.parse().timeout = None
                generate_bibi(experiment, generated_bibi, 'local', 0)
                self.assertEquals(pp.generate_cle.call_args_list[0][0][2], 600.0)

                p.parse().timeout = 500.0
                generate_bibi(experiment, generated_bibi, 'local', 0)
                self.assertEquals(pp.generate_cle.call_args_list[1][0][2], 500.0)

    def test_generate_experiment_control(self):
        """
        Test the experiment state machine control generation
        """
        with mock.patch('os.environ.get') as environ_get_mock:
            environ_get_mock.return_value = "testdir/"

            directory = os.path.split(__file__)[0]
            experiment = os.path.join(directory, 'ExDXMLExample.xml')

            paths = generate_experiment_control(experiment)
            self.assertEquals(len(paths), 0)

            directory = os.path.split(__file__)[0]
            experiment = os.path.join(directory, 'ExDXMLExample_with_sm.xml')

            paths = generate_experiment_control(experiment)
            self.assertEquals(len(paths), 3)
            expected = {"ControlSM1": "testdir/control_sm1.py",
                        "ControlSM2": "testdir/control_sm2.py",
                        "EvaluatingSM": "testdir/eval_sm1.py"}

            for name in expected:
                self.assertEquals(name in paths and expected[name], paths[name])

            environ_get_mock.assert_called_with("NRP_MODELS_DIRECTORY")


if __name__ == '__main__':
    unittest.main()
