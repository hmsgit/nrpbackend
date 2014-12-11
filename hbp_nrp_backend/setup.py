'''setup.py'''

# pylint: disable=F0401,E0611,W0142

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

import hbp_nrp_backend

from pip.req import parse_requirements
install_reqs = parse_requirements('requirements.txt')
from optparse import Option
options = Option("--workaround")
options.skip_requirements_regex = None
install_reqs = parse_requirements("./requirements.txt", options=options)
reqs = [str(ir.req) for ir in install_reqs]

config = {
    'description': 'Experiment Backend for HBP SP10',
    'author': 'hinkel',
    'url': 'https://bbpteam.epfl.ch/project/spaces/display/HSP10/Neurorobotics+Platform+Home',
    'author_email': 'bbp-dev-neuroroboticsplatform@groupes.epfl.ch',
    'version': hbp_nrp_backend.__version__,
    'install_requires': reqs,
    'packages': ['hbp_nrp_backend', 'hbp_nrp_backend.rest_server',
                 'hbp_nrp_backend.simulation_control',
                 'hbp_nrp_backend.bibi_config',
                 'hbp_nrp_backend.bibi_config.generated',
                 'hbp_nrp_backend.exd_config',
                 'hbp_nrp_backend.exd_config.generated'],
    'data_files': ['hbp_nrp_backend/bibi_config/*.pyt',
                   'hbp_nrp_backend/exd_config/*.pyt'],
    'scripts': [],
    'name': 'hbp_nrp_backend',
    'include_package_data': True,
}

setup(**config)
