# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
Unit tests for the service that retrieves experiments
"""

__author__ = "Bernd Eckstein"

import unittest
import os
import sys
import json
from flask import Response
from mock import patch, MagicMock
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server.__ExperimentService import get_experiment_basepath
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
from hbp_nrp_commons.generated import exp_conf_api_gen


PATH = os.path.split(__file__)[0]
EXPERIMENTS_PATH = os.path.join(PATH, 'experiments')


class TestExperimentService1(unittest.TestCase):

    @patch("hbp_nrp_backend.rest_server.__ExperimentService.os")
    def test_get_experiment_basepath_ok(self, mock_os):
        mock_os.environ.get.return_value = "/test1"
        self.assertEqual("/test1", get_experiment_basepath())

    @patch("hbp_nrp_backend.rest_server.__ExperimentService.os")
    def test_get_experiment_basepath_error(self, mock_os):
        mock_os.environ.get.return_value = None
        self.assertRaises(NRPServicesGeneralException, get_experiment_basepath)

if __name__ == '__main__':
    unittest.main()
