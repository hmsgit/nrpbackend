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
    'author': 'HBP Neurorobotics',
    'url': 'http://neurorobotics.net',
    'author_email': 'neurorobotics@humanbrainproject.eu',
    'version': hbp_nrp_backend.__version__,
    'install_requires': reqs,
    'packages': ['hbp_nrp_backend', 'hbp_nrp_backend.rest_server',
                 'hbp_nrp_backend.collab_interface',
                 'hbp_nrp_backend.simulation_control',
                 'hbp_nrp_backend.exd_config',
                 'hbp_nrp_backend.exd_config.generated',
                 'hbp_nrp_backend.cle_interface'],
    'scripts': [],
    'name': 'hbp_nrp_backend',
    'include_package_data': True,
}

setup(**config)
