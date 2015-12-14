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
from mock import patch, MagicMock

rospy.ServiceProxy = MagicMock(return_value=MagicMock())


class TestExperimentConfigurationScript(unittest.TestCase):
    """
    Test the generation of the ExD config script
    """

    def setUp(self):
        self.directory = os.path.dirname(__file__)

    def test_initialize_experiment(self):
        """
        Tests the experiment initialization.
        """

        directory = os.path.split(__file__)[0]
        experiment = os.path.join(directory, 'ExDXMLExample.xml')
        self.assertIsInstance(initialize_experiment(experiment, os.path.dirname(__file__),
                                                    0, 'local'), ROSCLEClient)

    @patch('os.environ.get')
    def test_generate_bibi(self, environ_get_mock):
        """
        Test the generation.
        """

        experiment = os.path.join(self.directory, 'ExDXMLExample.xml')
        generated_bibi = os.path.join(self.directory, 'generated_bibi.py')
        environ_get_mock.return_value = self.directory

        with patch('hbp_nrp_backend.exd_config.experiment_configuration_script'
                        '.exp_conf_api_gen') as p:
            with patch('hbp_nrp_backend.exd_config.experiment_configuration_script'
                            '.bibi_configuration_script') as pp:
                document = p.CreateFromDocument()
                document.timeout = None
                document.bibiConf = MagicMock(src='bibi_configuration.xml')
                generate_bibi(experiment, generated_bibi, 'local', 0, collab=False)
                self.assertEquals(pp.generate_cle.call_args_list[0][0][2], 600.0)
                bibi_configuration_path = os.path.join(
                    self.directory,
                    document.bibiConf.src
                )
                self.assertEquals(
                    pp.generate_cle.call_args_list[0][0][5], # tf_path
                    os.path.dirname(bibi_configuration_path)
                )
                document.timeout = 500.0
                generate_bibi(experiment, generated_bibi, 'local', 0)
                self.assertEquals(pp.generate_cle.call_args_list[1][0][2], 500.0)
                self.assertEquals(
                    pp.generate_cle.call_args_list[1][0][5], # tf_path
                    self.directory
                )
                generate_bibi(experiment, generated_bibi, 'local', 0, tf_path='some_path')
                self.assertEquals(
                    pp.generate_cle.call_args_list[2][0][5], # tf_path
                    'some_path'
                )

    def test_generate_experiment_control(self):
        """
        Test the experiment state machine control generation
        """
        with patch('os.environ.get') as environ_get_mock:
            environ_get_mock.return_value = "testdir/"

            experiment = os.path.join(self.directory, 'ExDXMLExample.xml')

            paths = generate_experiment_control(experiment, self.directory)
            self.assertEquals(len(paths), 0)

            experiment = os.path.join(self.directory, 'ExDXMLExample_with_sm.xml')

            paths = generate_experiment_control(experiment, self.directory)
            self.assertEquals(len(paths), 3)
            expected = {"ControlSM1": os.path.join(self.directory, "control_sm1.py"),
                        "ControlSM2": os.path.join(self.directory, "control_sm2.py"),
                        "EvaluatingSM": os.path.join(self.directory, "eval_sm1.py")}

            for name in expected:
                self.assertEquals(name in paths and expected[name], paths[name])


if __name__ == '__main__':
    unittest.main()
