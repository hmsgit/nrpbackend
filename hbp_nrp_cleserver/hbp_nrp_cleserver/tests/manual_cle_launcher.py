"""
This module is meant as a debugging helper to manually start the cle
"""

from hbp_nrp_commons.generated import exp_conf_api_gen as exd
from hbp_nrp_commons.generated import bibi_api_gen as bibi
from hbp_nrp_cleserver.server import CLELauncher
from os import environ, path

__author__ = 'Georg Hinkel'

def create_cle_launcher(config):
    models_path = environ.get('NRP_MODELS_DIRECTORY')

    with open(path.join(models_path, config)) as exd_file:
        exd_config = exd.CreateFromDocument(exd_file.read())

    with open(path.join(models_path, exd_config.bibiConf)) as bibi_file:
        bibi_config = bibi.CreateFromDocument(bibi_file.read())

    cle_launcher = CLELauncher(exd_config, bibi_config, models_path, 'local', 0)
    return cle_launcher, exd_config, bibi_config