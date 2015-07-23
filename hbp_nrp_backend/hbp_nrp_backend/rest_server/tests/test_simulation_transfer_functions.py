"""
Unit tests for the service that retrieves transfer function sources
"""

__author__ = 'DanielPeppicelli, LucGuyot'

import hbp_nrp_backend
from hbp_nrp_backend.rest_server import app
from mock import patch, MagicMock
import unittest
import json


class TestSimulationTransferFunctions(unittest.TestCase):
    @patch('hbp_nrp_backend.rest_server.__SimulationTransferFunctions._get_simulation_or_abort')
    def test_simulation_transfer_functions_get(self, mocked_get_simulation_or_abort):
        mocked_simulation = MagicMock()
        mocked_simulation.cle = MagicMock()
        transfer_functions = {"transfer_functions": ["some code", "more code"]}
        mocked_simulation.cle.get_simulation_transfer_functions = MagicMock(return_value=transfer_functions)
        mocked_get_simulation_or_abort.return_value = mocked_simulation
        client = app.test_client()
        response = client.get('/simulation/0/transfer-functions')
        self.assertEqual(mocked_simulation.cle.get_simulation_transfer_functions.call_count, 1)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data.strip(), json.dumps(transfer_functions))


if __name__ == '__main__':
    unittest.main()
