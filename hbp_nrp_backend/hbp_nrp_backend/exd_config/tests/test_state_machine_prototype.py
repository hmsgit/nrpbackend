"""
Test to run the default state machine
"""

__author__ = 'Alessandro Ambrosano'

import unittest
from hbp_nrp_backend.exd_config.state_machine_prototype import StateMachineTimedColorChange


class TestScript(unittest.TestCase):
    """
    Test to run the default state machine
    """

    def test_populate_state_machine_timed_color_change(self):
        state_list = StateMachineTimedColorChange.populate()
        self.assertIsNotNone(state_list)

if __name__ == '__main__':
    unittest.main()