"""
Unit tests for the service that retrieves transfer function sources
"""

__author__ = 'DanielPeppicelli, LucGuyot'

import unittest
import json
from mock import patch, MagicMock
from hbp_nrp_backend.rest_server.__SimulationTransferFunctions import get_tf_name
from hbp_nrp_backend.rest_server.tests import RestTest


class TestSimulationTransferFunctions(RestTest):
    def test_get_tf_name(self):
        self.assertEqual(None, get_tf_name(" not valid :)"))
        self.assertEqual("tf1", get_tf_name("def tf1():\n return"))
        self.assertEqual("tf1", get_tf_name("def tf1(a,b,c):\n return"))
        self.assertEqual("tf1", get_tf_name("def tf1  (a,b,c):\n return"))

    @patch('hbp_nrp_backend.rest_server.__SimulationTransferFunctions._get_simulation_or_abort')
    def test_simulation_transfer_functions_get(self, mocked_get_simulation_or_abort):
        mocked_simulation = MagicMock()
        mocked_simulation.cle = MagicMock()
        transfer_functions_list = ["def tf1(unused):\n return", "def tf2():\n return"]
        mocked_simulation.cle.get_simulation_transfer_functions = MagicMock(return_value=transfer_functions_list)
        mocked_get_simulation_or_abort.return_value = mocked_simulation
        response = self.client.get('/simulation/0/transfer-functions')
        self.assertEqual(mocked_simulation.cle.get_simulation_transfer_functions.call_count, 1)
        self.assertEqual(response.status_code, 200)
        transfer_functions = {
            "data":
            {"tf1": "def tf1(unused):\n return", "tf2": "def tf2():\n return"}
        }
        self.assertEqual(response.data.strip(), json.dumps(transfer_functions))

if __name__ == '__main__':
    unittest.main()
