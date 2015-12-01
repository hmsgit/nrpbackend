"""
This package contains the python code to run the REST web server and supportive tooling
"""

from hbp_nrp_backend.version import VERSION as __version__  # pylint: disable=unused-import
from hbp_nrp_backend import config
import os

__author__ = 'GeorgHinkel'

setting_str = os.environ['APP_SETTINGS']
# APP_SETTINGS is of the form config.MySpecificConfig
config_class_str = setting_str.split('.', 1)[1]
# Application configuration, shared accross all hbp_nrp_backend files
hbp_nrp_backend_config = getattr(config, config_class_str)
