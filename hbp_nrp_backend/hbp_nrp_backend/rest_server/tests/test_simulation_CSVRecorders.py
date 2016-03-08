"""
Unit tests for the simulation SDF services
"""
__author__ = 'LucGuyot'

import os
import json
from mock import patch, MagicMock
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server.tests import RestTest


class TestSimulationCSVRecorders(RestTest):
    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', None, 'default-owner', 'created'))
        simulations[0].cle = MagicMock()
        self.files = [
          MagicMock(name='left_wheel_join_position.csv', temporary_path='/tmp/csv_recorders/left_wheel_join_position.csv'),
          MagicMock(name='right_wheel_join_position.csv', temporary_path='/tmp/csv_recorders/right_wheel_join_position.csv')
        ]
        simulations[0].cle.get_simulation_CSV_recorders_files = MagicMock(return_value=self.files)
        patch_CollabClient = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

    @patch('hbp_nrp_backend.rest_server.__SimulationCSVRecorders.get_date_and_time_string')
    def test_simulation_CSV_recorders_put_OK(self, mock_get_date_and_time_string):
        time = '2016-04-11_13-15-34'
        mock_get_date_and_time_string.return_value = time
        context_id = '123456'
        sim_id = 0
        subfolder_uuid = '123-456-789-e'
        self.mock_collabClient_instance.populate_subfolder_in_collab.return_value = subfolder_uuid
        response = self.client.put('/simulation/' + str(sim_id) + '/' + context_id + '/csv-recorders')
        self.assertEqual(self.mock_collabClient_instance.populate_subfolder_in_collab.call_count, 1)
        self.assertEqual(self.mock_collabClient_instance.populate_subfolder_in_collab.call_args[0][0],
                   'csv_records_' + time)
        self.assertEqual(self.mock_collabClient_instance.populate_subfolder_in_collab.call_args[0][1],
                    self.files)
        self.assertEqual(self.mock_collabClient_instance.populate_subfolder_in_collab.call_args[0][2],
                    'text/csv')
        self.assertEqual(response.status_code, 200)

    def test_simulation_CSV_recorders_put_sim_not_found(self):
        response = self.client.put('/simulation/1/123456/csv-recorders')
        self.assertEqual(404, response.status_code)