'''setup.py'''
# pylint: disable=R0801
import os

from setuptools import setup

from pip.req import parse_requirements
from optparse import Option

import hbp_nrp_cleserver


def parse_reqs(reqs_file):
    ''' parse the requirements '''
    options = Option('--workaround')
    options.skip_requirements_regex = None
    install_reqs = parse_requirements(reqs_file, options=options)
    return [str(ir.req) for ir in install_reqs]


BASEDIR = os.path.dirname(os.path.abspath(__file__))
REQS = parse_reqs(os.path.join(BASEDIR, 'requirements.txt'))

EXTRA_REQS_PREFIX = 'requirements_'
EXTRA_REQS = {}
for file_name in os.listdir(BASEDIR):
    if not file_name.startswith(EXTRA_REQS_PREFIX):
        continue
    base_name = os.path.basename(file_name)
    (extra, _) = os.path.splitext(base_name)
    extra = extra[len(EXTRA_REQS_PREFIX):]
    EXTRA_REQS[extra] = parse_reqs(file_name)

config = {
    'description': 'CLE Simulation Factory for HBP SP10',
    'author': 'HBP Neurorobotics',
    'url': 'http://neurorobotics.net',
    'author_email': 'neurorobotics@humanbrainproject.eu',
    'version': hbp_nrp_cleserver.__version__,
    'install_requires': REQS,
    'extras_require': EXTRA_REQS,
    'packages': ['hbp_nrp_cleserver',
                 'hbp_nrp_cleserver.bibi_config',
                 'hbp_nrp_cleserver.server'],
    'scripts': [],
    'name': 'hbp-nrp-cleserver',
    'include_package_data': True,
}

setup(**config)
