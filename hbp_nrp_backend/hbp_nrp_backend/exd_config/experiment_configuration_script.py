"""
Script to run Experiment from ExperimentDesigner Configuration File
"""
from hbp_nrp_backend.exd_config.generated import generated_experiment_api

__author__ = 'Lorenzo Vannucci'

from hbp_nrp_backend.bibi_config import bibi_configuration_script
import os
import jinja2

# pylint: disable=E1103
# pylint infers the wrong type for config


def generate_experiment(experiment_conf, script_file_name):
    """
    Generates Code to run the experiment based on the given configuration file
    :param bibi_conf: The Experiment Designer configuration
    :param script_file_name: The file name of the script to be generated, \
    including .py
    """

    # parse experiment configuration
    config = generated_experiment_api.parse(experiment_conf, silence=True)

    # generate bibi script
    bibi_conf = config.bibiConf
    basename = os.path.splitext(script_file_name)[0]
    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        path = os.path.dirname(experiment_conf)
    if not path == '':
        complete_bibi_conf = os.path.join(path, bibi_conf)
    else:
        complete_bibi_conf = bibi_conf
    bibi_configuration_script.generate_cle(complete_bibi_conf,
                                           basename + '_bibi.py')

    # generate experiment script
    env = config.environmentModel

    names = {'model': env,
             'bibi_script': basename + '_bibi.py'}

    template_path = os.path.join(os.path.split(__file__)[0],
                                 'experiment_template.pyt')
    template_file = open(template_path, 'r')
    template = jinja2.Template(template_file.read())
    template_file.close()

    output_file = open(script_file_name, 'w')
    output_file.write(template.render(names))
    output_file.close()
