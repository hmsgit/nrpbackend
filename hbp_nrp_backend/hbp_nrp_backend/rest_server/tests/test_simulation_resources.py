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
Unit tests for the simulation resources
"""
__author__ = 'Yves Schmid Dornbierer'


import os
import unittest
from mock import patch
import json
from hbp_nrp_backend.rest_server.tests import RestTest


class MockedSimulationLifeCycle:

    experiment_path = ""
    sim_dir = ""


class MockedSimulation:

    lifecycle = MockedSimulationLifeCycle()
    ctx_id = None
    experiment_id = False


class TestSimulationResources(RestTest):

    def setUp(self):
        self.test_directory = os.path.split(__file__)[0]
        self.path_can_view = patch('hbp_nrp_backend.__UserAuthentication.UserAuthentication.can_view')
        self.path_can_view.start().return_value = True

    def tearDown(self):
        self.path_can_view.stop()

    @patch('hbp_nrp_backend.rest_server.__SimulationResources._get_simulation_or_abort')
    def test_get_simulation_resources(self, patch_SimulationControl):

        sim = MockedSimulation()
        sim.lifecycle.experiment_path = os.path.join(
            self.test_directory, "experiments", "experiment_data", "testsimulationresources.exc")

        expected_file_list = ['test.json', 'brain_visualizer/brainvisualiserdata.json']

        sim.lifecycle.sim_dir = os.path.dirname(sim.lifecycle.experiment_path)

        patch_SimulationControl.return_value = sim

        sim.private = None

        resources = self.client.get('/simulation/0/resources')
        resources_data = json.loads(resources.data.strip())['resources']

        self.assertEqual(resources.status_code, 200)

        for datum in resources_data:
            self.assertIn(datum['file'][datum['file_offset']:], expected_file_list)

    @patch('hbp_nrp_backend.rest_server.__SimulationResources._get_simulation_or_abort')
    def test_no_simulation_xml_resources(self, patch_SimulationControl):

        sim = MockedSimulation()
        sim.lifecycle.experiment_path = ""

        sim.lifecycle.sim_dir = os.path.dirname(sim.lifecycle.experiment_path)

        patch_SimulationControl.return_value = sim

        resources = self.client.get('/simulation/0/resources')

        self.assertEqual(resources.status_code, 404)

    @patch('hbp_nrp_backend.rest_server.__SimulationResources._get_simulation_or_abort')
    def test_cant_find_linked_simulation_resources(self, patch_SimulationControl):

        sim = MockedSimulation()
        sim.lifecycle.experiment_path = os.path.join(
            self.test_directory, "experiments", "experiment_data", "testsimulationresources.exc")

        sim.lifecycle.sim_dir = "wrong_folder"

        patch_SimulationControl.return_value = sim

        resources = self.client.get('/simulation/0/resources')

        self.assertEqual(resources.status_code, 404)

    @patch('hbp_nrp_backend.rest_server.__SimulationResources._get_simulation_or_abort')
    def test_no_linked_simulation_resources(self, patch_SimulationControl):

        sim = MockedSimulation()
        sim.lifecycle.experiment_path = os.path.join(
            self.test_directory, "experiments", "experiment_data", "test_1.exc")

        sim.lifecycle.sim_dir = os.path.dirname(sim.lifecycle.experiment_path)

        patch_SimulationControl.return_value = sim

        sim.private = None

        resources = self.client.get('/simulation/0/resources')

        self.assertEqual(resources.status_code, 200)


if __name__ == '__main__':
    unittest.main()
