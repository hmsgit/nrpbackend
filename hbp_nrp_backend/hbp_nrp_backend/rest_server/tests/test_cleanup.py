"""
Tests for cleanup.py
"""

import unittest
from mock import patch, MagicMock
import hbp_nrp_backend.rest_server.cleanup as cleanup
from datetime import datetime, timedelta
import pytz


class TestCleanup(unittest.TestCase):
    @patch('hbp_nrp_backend.rest_server.cleanup.simulations')
    def test_remove_old_simulations(self, simulations):
        timezone = pytz.timezone('Europe/Zurich')

        sim = MagicMock()
        sim.kill_datetime = datetime.now(timezone) - timedelta(days=2)
        sim.sim_id = 5
        sim.state = 'running'

        sim2 = MagicMock()
        sim2.kill_datetime = datetime.now(timezone) + timedelta(days=2)
        sim2.sim_id = 5
        sim2.state = 'running'

        simulations.__iter__.return_value = [sim, sim2]
        cleanup.remove_old_simulations()
        assert sim.state == 'stopped'
        assert sim2.state == 'running'


if __name__ == '__main__':
    unittest.main()
