"""
This package contains the python code to run the REST web server and supportive tooling
"""

from hbp_nrp_backend.version import VERSION as __version__  # pylint: disable=unused-import
from hbp_nrp_backend import config
import os

__author__ = 'GeorgHinkel'

setting_str = os.environ.get('APP_SETTINGS')
# The APP_SETTINGS environment variable allows you to easily switch configurations.
# Standard configurations are listed in config.py.
# APP_SETTINGS defaults to config.DeploymentConfig on a puppet-managed server
# (see nrp-services/nrp-services-env.sh from server-scripts repo).
# Some other valid config objects: config.TestConfig, config.LocalConfig.
if setting_str is not None:
    config_class_str = setting_str.split('.', 1)[1]
    # Application configuration object, shared accross all hbp_nrp_backend files
    hbp_nrp_backend_config = getattr(config, config_class_str)
