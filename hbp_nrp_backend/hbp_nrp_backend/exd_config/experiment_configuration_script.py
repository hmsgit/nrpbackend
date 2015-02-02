"""
Script to run Experiment from ExperimentDesigner Configuration File
"""
from hbp_nrp_backend.exd_config.generated import generated_experiment_api

__author__ = 'Lorenzo Vannucci'

from hbp_nrp_backend.bibi_config import bibi_configuration_script
import os
import imp
import multiprocessing
from hbp_nrp_cle.cle.ROSCLEWrapper import ROSCLEClient
from hbp_nrp_cle.robotsim.GazeboLoadingHelper import load_gazebo_world_file, empty_gazebo_world

# pylint: disable=E1103
# pylint infers the wrong type for config


def generate_bibi(experiment_conf, bibi_script_file_name):
    """
    Generates Code to run the Brain interface and Body integrator based on the
    given experiment configuration file.
    :param experiment_conf: The Experiment Designer configuration. The code will
                            search in the folder set in NRP_MODELS_DIRECTORY
                            environment variable for this file. relative path from
                            there must be included.
    :param bibi_script_file_name: The file name of the script to be generated, \
                                  including .py and the complete path.
    """

    # parse experiment configuration
    experiment = generated_experiment_api.parse(experiment_conf, silence=True)

    # retrieve the bibi configuration file name.
    bibi_conf = os.path.join(_get_basepath(experiment_conf),
                             experiment.bibiConf)

    bibi_configuration_script.generate_cle(bibi_conf,
                                           bibi_script_file_name)


def initialize_experiment(experiment_conf, bibi_script_file_name):
    """
    Initialize experiment based on generated code by generate_bibi.
    :param experiment_conf: The Experiment Designer configuration. The code will
                            search in the folder set in NRP_MODELS_DIRECTORY
                            environment variable for this file. relative path from
                            there must be included.
    :param bibi_script_file_name: The file name of the script to be generated, \
                                  including .py and the complete path.
    """

    # Join any remaining processes. From the doc:
    # Calling this has the side effect of "joining" any processes which have already finished.
    active_children_processes = multiprocessing.active_children()
    if (len(active_children_processes) > 0):
        # As long as we do not spawn other processes than the experiment
        # scripts, this check is ok. It avoids us to save the list of spawned
        # processes somewhere.
        raise Exception("Last experiment has not been correctly terminated.")

    # parse experiment configuration to get the environment to spawn.
    experiment = generated_experiment_api.parse(experiment_conf, silence=True)
    bibi_script = imp.load_source('bibi_configuration', bibi_script_file_name)
    empty_gazebo_world()
    load_gazebo_world_file(experiment.environmentModel.location)
    p = multiprocessing.Process(target=bibi_script.cle_function)
    p.start()
    return ROSCLEClient()


def _get_basepath(configuration_file=None):
    """
    :return the basepath for retrieving and storing scripts / \
    configuration files. There are three possible cases. They
    are evaluated in the following order: \
    1. The NRP_MODELS_DIRECTORY variable is set, then return the \
       content of this variable.
    2. The configuration_file argument is set, then return the \
       basepath of this file.
    3. Return None.
    """
    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        path = os.path.dirname(configuration_file)
    return path
