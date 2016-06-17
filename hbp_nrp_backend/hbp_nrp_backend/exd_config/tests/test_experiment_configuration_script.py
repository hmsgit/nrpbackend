"""
Test to run the ExD configuration script
"""

__author__ = 'Lorenzo Vannucci'

from hbp_nrp_backend.exd_config.experiment_configuration_script \
    import generate_experiment_control, initialize_experiment
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClient
from hbp_nrp_commons.generated import exp_conf_api_gen
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
        experiment_path = os.path.join(directory, 'ExDXMLExample.xml')

        with open(experiment_path) as exd_file:
            experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())

        self.assertIsInstance(initialize_experiment(experiment, experiment_path, os.path.dirname(__file__),
                                                    0, 'local'), ROSCLEClient)

    def test_generate_experiment_control(self):
        """
        Test the experiment state machine control generation
        """
        with patch('os.environ.get') as environ_get_mock:
            environ_get_mock.return_value = "testdir/"

            experiment_path = os.path.join(self.directory, 'ExDXMLExample.xml')

            with open(experiment_path) as exd_file:
                experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())

            paths = generate_experiment_control(experiment, self.directory)
            self.assertEquals(len(paths), 0)

            experiment_path = os.path.join(self.directory, 'ExDXMLExample_with_sm.xml')

            with open(experiment_path) as exd_file:
                experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())

            paths = generate_experiment_control(experiment, self.directory)
            self.assertEquals(len(paths), 3)
            expected = {"ControlSM1": os.path.join(self.directory, "control_sm1.py"),
                        "ControlSM2": os.path.join(self.directory, "control_sm2.py"),
                        "EvaluatingSM": os.path.join(self.directory, "eval_sm1.py")}

            for name in expected:
                self.assertEquals(name in paths and expected[name], paths[name])


if __name__ == '__main__':
    unittest.main()
