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
Unit tests for the __SimulationReset module.
"""

__author__ = 'Alessandro Ambrosano'

import unittest
import mock
import json
import os

from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.simulation_control import simulations, Simulation
from cle_ros_msgs.srv import ResetSimulationRequest

from mock import patch, MagicMock, Mock

PATH = os.path.split(__file__)[0]


class TestSimulationReset(RestTest):

    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiments/experiment_data/test_1.exc', None, 'default-owner', 'created'))
        simulations.append(Simulation(1, 'experiments/experiment_data/test_1.exc', None, 'im-not-the-owner', 'created'))

    def test_put_reset(self):
        simulations[0].cle = mock.MagicMock()

        # Correct request
        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        self.assertEqual(200, response.status_code)
        simulations[0].cle.reset.assert_called()

        # Invalid request, too much parameters
        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE,
            'randomInvalidParameter': False
        }))
        self.assertEqual(400, response.status_code)

        # Invalid request, missing parameters
        response = self.client.put('/simulation/0/reset', data=json.dumps({}))
        self.assertEqual(400, response.status_code)

        # This simulation doesn't exist
        response = self.client.put('/simulation/2/reset')
        self.assertEqual(404, response.status_code)

        # I'm not the owner of this one
        response = self.client.put('/simulation/1/reset')
        self.assertEqual(401, response.status_code)

        # Now the request is fine, but something goes wrong out of the backend reach
        simulations[0].cle.reset.side_effect = ROSCLEClientException()
        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        self.assertEqual(500, response.status_code)

    @patch("hbp_nrp_backend.rest_server.__SimulationReset.get_experiment_basepath")
    @patch("hbp_nrp_backend.rest_server.__SimulationReset.get_model_basepath")
    def test_reset_is_called_properly(self, mock_get_model_path, mock_get_experiment_path):
        simulations[0].cle = mock.MagicMock()
        simulations[0].cle.set_simulation_transfer_function.return_value = None

        mock_get_experiment_path.return_value = PATH
        mock_get_model_path.return_value = os.path.join(PATH, 'models')

        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_ROBOT_POSE
        }))
        simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_ROBOT_POSE)

        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_FULL
        }))
        # this test will fail once the reset functionality is working again (see issue report NRRPLT-4860)
        # at that moment, 500 should be replaced by 200 and RESET_ROBOT_POSE should be replaced by RESET_FULL in the next lines
        self.assertEqual(500, response.status_code)
        simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_ROBOT_POSE)

        response = self.client.put('/simulation/0/reset', data=json.dumps({
            'resetType': ResetSimulationRequest.RESET_WORLD
        }))
        simulations[0].cle.reset.assert_called_with(ResetSimulationRequest.RESET_WORLD)


if __name__ == '__main__':
    unittest.main()
