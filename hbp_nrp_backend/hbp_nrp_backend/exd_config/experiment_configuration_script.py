"""
Script to run Experiment from ExperimentDesigner Configuration File
"""
from hbp_nrp_backend.exd_config.generated import exp_conf_api_gen
from hbp_nrp_cle.bibi_config import bibi_configuration_script
import os
import logging
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClient
from hbp_nrp_backend.cle_interface.ROSCLESimulationFactoryClient \
    import ROSCLESimulationFactoryClient

__author__ = 'Lorenzo Vannucci, Daniel Peppicelli'

logger = logging.getLogger(__name__)


# pylint: disable=E1103
# pylint infers the wrong type for config


def generate_bibi(experiment_conf, bibi_script_file_name, gzserver_host, sim_id, models_path):
    """
    Generates Code to run the Brain interface and Body integrator based on the
    given experiment configuration file.

    :param experiment_conf: The Experiment Designer configuration. The code will
                            search in the folder set in NRP_MODELS_DIRECTORY
                            environment variable for this file. relative path from
                            there must be included.
    :param bibi_script_file_name: The file name of the script to be generated, \
                                  including .py and the complete path.

    :param gzserver_host: The host where the gzserver will run, local for local machine
        lugano for remote Lugano viz cluster.
    """

    logger.info(("Generating BIBI configuration for experiment {0} to {1}" +
                 "(gz services running on {2})")
                .format(experiment_conf, bibi_script_file_name, gzserver_host))

    with open(experiment_conf) as exd_file:

        # parse experiment configuration
        experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())

    # retrieve the bibi configuration file name.
    bibi_conf = os.path.join(_get_basepath(experiment_conf),
                             experiment.bibiConf.src)

    # set timeout to 10 minutes, if not specified by default
    if experiment.timeout is None:
        timeout = 600.0
    else:
        timeout = experiment.timeout

    bibi_configuration_script.generate_cle(bibi_conf,
                                           bibi_script_file_name,
                                           timeout,
                                           gzserver_host,
                                           sim_id,
                                           models_path)


def generate_experiment_control(experiment_conf):
    """
    Currently just parses the experiment configuration to extract the paths to the experiment
    controlling state machine scripts.

    :param experiment_conf: The Experiment Designer configuration. The code will
                            search in the folder set in NRP_MODELS_DIRECTORY
                            environment variable for this file. relative path from
                            there must be included.
    :return: A dictionary mapping the specified unique state machine name to the according file
             path of the python script
    """
    # parse experiment configuration
    with open(experiment_conf) as exd_file:
        experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())

    state_machine_paths = {}

    #
    # BEGIN Actually generate the state machines from config here
    #
    if experiment.experimentControl is not None:
        state_machine_paths.update({sm.id: os.path.join(_get_basepath(experiment_conf), sm.src)
                                    for sm in experiment.experimentControl.stateMachine
                                    if isinstance(sm, exp_conf_api_gen.SMACHStateMachine)})

    if experiment.experimentEvaluation is not None:
        state_machine_paths.update({sm.id: os.path.join(_get_basepath(experiment_conf), sm.src)
                                    for sm in experiment.experimentEvaluation.stateMachine
                                    if isinstance(sm, exp_conf_api_gen.SMACHStateMachine)})
        #
        # END Actually generate the state machines from config here
        #

    return state_machine_paths


def initialize_experiment(experiment_conf, generated_cle_script_file, sim_id):
    """
    Initialize experiment based on generated code by generate_bibi.

    :param experiment_conf: The Experiment Designer configuration. The code will
        search in the folder set in NRP_MODELS_DIRECTORY
        environment variable for this file. relative path from there must be included.

    :param generated_cle_script_file: The file name of the generated cle script,
        including .py and the complete path.
    """

    # parse experiment configuration to get the environment to spawn.
    logger.info("Requesting simulation resources")
    with open(experiment_conf) as exd_file:
        experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())
    simulation_factory_client = ROSCLESimulationFactoryClient()
    simulation_factory_client.start_new_simulation(
        str(experiment.environmentModel.src),
        os.path.join(os.getcwd(), generated_cle_script_file))
    return ROSCLEClient(sim_id)


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
