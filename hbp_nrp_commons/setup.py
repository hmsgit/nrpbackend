'''setup.py'''
import os

from setuptools import setup

import pip
from optparse import Option

import hbp_nrp_commons


def parse_reqs(reqs_file):
    ''' parse the requirements '''
    options = Option("--workaround")
    options.skip_requirements_regex = None
    # Hack for old pip versions
    if pip.__version__.startswith('10.'):
        # Versions greater or equal to 10.x don't rely on pip.req.parse_requirements
        install_reqs = list(val.strip() for val in open(reqs_file))
        reqs = install_reqs
    elif pip.__version__.startswith('1.'):
        # Versions 1.x rely on pip.req.parse_requirements
        # but don't require a "session" parameter
        from pip.req import parse_requirements # pylint:disable=no-name-in-module, import-error
        install_reqs = parse_requirements(reqs_file, options=options)
        reqs = [str(ir.req) for ir in install_reqs]
    else:
        # Versions greater than 1.x but smaller than 10.x rely on pip.req.parse_requirements
        # and requires a "session" parameter
        from pip.req import parse_requirements # pylint:disable=no-name-in-module, import-error
        from pip.download import PipSession  # pylint:disable=no-name-in-module, import-error
        options.isolated_mode = False
        install_reqs = parse_requirements(  # pylint:disable=unexpected-keyword-arg
            reqs_file,
            session=PipSession,
            options=options
        )
        reqs = [str(ir.req) for ir in install_reqs]
    return reqs


BASEDIR = os.path.dirname(os.path.abspath(__file__))
REQS = parse_reqs(os.path.join(BASEDIR, 'requirements.txt'))

# ensure we install numpy before the main list of requirements, ignore
# failures if numpy/cython are not requirements and just proceed (futureproof)
try:
    cython_req = next(r for r in REQS if r.startswith('cython'))
    numpy_req = next(r for r in REQS if r.startswith('numpy'))
    if pip.__version__.startswith('10.'):
        import subprocess
        subprocess.check_call(
          ["python", '-m', 'pip', 'install', "--no-clean", "--user", cython_req, numpy_req]
        )
    else:
        pip.main(['install', '--no-clean', cython_req, numpy_req]) # pylint:disable=no-member
# pylint: disable=bare-except
except:
    pass

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
    'description': 'NRP Backend shared functionality',
    'author': 'HBP Neurorobotics',
    'url': 'http://neurorobotics.net',
    'author_email': 'neurorobotics-support@humanbrainproject.eu',
    'version': hbp_nrp_commons.__version__,
    'install_requires': REQS,
    'extras_require': EXTRA_REQS,
    'packages': ['hbp_nrp_commons',
                 'hbp_nrp_commons.generated',
                 'hbp_nrp_commons.cluster'],
    'scripts': [],
    'name': 'hbp-nrp-commons',
    'include_package_data': True,
}

setup(**config)
