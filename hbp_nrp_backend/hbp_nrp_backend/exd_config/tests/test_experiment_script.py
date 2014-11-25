"""
Test to run the ExD configuration script
"""

__author__ = 'Lorenzo Vannucci'

from hbp_nrp_backend.exd_config.experiment_configuration_script \
    import generate_experiment
import unittest
import os


class TestScript(unittest.TestCase):
    """
    Test the generation of the ExD config script
    """

    def test_experiment_script(self):
        """
        Test the generation.
        """
        directory = os.path.split(__file__)[0]
        experiment = os.path.join(directory, 'ExDXMLExample.xml')
        generate_experiment(experiment,
                            os.path.join(directory, 'experiment.py'))


if __name__ == '__main__':
    unittest.main()
