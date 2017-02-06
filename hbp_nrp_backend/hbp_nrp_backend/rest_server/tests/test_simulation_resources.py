"""
Unit tests for the simulation resources
"""
__author__ = 'Yves Schmid Dornbierer'


import os
from mock import patch
from hbp_nrp_backend.rest_server.tests import RestTest


class MockedSimulationLifeCycle:

    experiment_path = ""
    simulation_root_folder = ""


class MockedSimulation:

    lifecycle = MockedSimulationLifeCycle()
    context_id = False


class TestSimulationResources(RestTest):

    def setUp(self):
        self.test_directory = os.path.split(__file__)[0]

    @patch('hbp_nrp_backend.rest_server.__SimulationResources._get_simulation_or_abort')
    def test_get_simulation_resources(self, patch_SimulationControl):

        sim = MockedSimulation()
        sim.lifecycle.experiment_path = os.path.join(
            self.test_directory, "experiments", "experiment_data", "testsimulationresources.exc")

        sim.lifecycle.simulation_root_folder = os.path.dirname(sim.lifecycle.experiment_path)

        patch_SimulationControl.return_value = sim

        resources = self.client.get('/simulation/0/resources')

        self.assertEqual(resources.status_code, 200)

    @patch('hbp_nrp_backend.rest_server.__SimulationResources._get_simulation_or_abort')
    def test_no_simulation_xml_resources(self, patch_SimulationControl):

        sim = MockedSimulation()
        sim.lifecycle.experiment_path = ""

        sim.lifecycle.simulation_root_folder = os.path.dirname(sim.lifecycle.experiment_path)

        patch_SimulationControl.return_value = sim

        resources = self.client.get('/simulation/0/resources')

        self.assertEqual(resources.status_code, 404)

    @patch('hbp_nrp_backend.rest_server.__SimulationResources._get_simulation_or_abort')
    def test_cant_find_linked_simulation_resources(self, patch_SimulationControl):

        sim = MockedSimulation()
        sim.lifecycle.experiment_path = os.path.join(
            self.test_directory, "experiments", "experiment_data", "testsimulationresources.exc")

        sim.lifecycle.simulation_root_folder = "wrong_folder"

        patch_SimulationControl.return_value = sim

        resources = self.client.get('/simulation/0/resources')

        self.assertEqual(resources.status_code, 404)

    @patch('hbp_nrp_backend.rest_server.__SimulationResources._get_simulation_or_abort')
    def test_no_linked_simulation_resources(self, patch_SimulationControl):

        sim = MockedSimulation()
        sim.lifecycle.experiment_path = os.path.join(
            self.test_directory, "experiments", "experiment_data", "test_1.exc")

        sim.lifecycle.simulation_root_folder = os.path.dirname(sim.lifecycle.experiment_path)

        patch_SimulationControl.return_value = sim

        resources = self.client.get('/simulation/0/resources')

        self.assertEqual(resources.status_code, 200)
