"""
Test to run the default state machine
"""

__author__ = 'Alessandro Ambrosano'

import unittest
from hbp_nrp_backend.exd_config.default_state_machine import DefaultState, DefaultStateMachine


class TestScript(unittest.TestCase):
    """
    Test to run the default state machine
    """

    def test_execute_default_state(self):
        ds = DefaultState()
        self.assertIsNotNone(ds._outcomes)
        self.assertGreater(len(ds._outcomes), 0)
        result = ds.execute(None)
        self.assertEqual(result, 'FINISHED')

    def test_populate_default_state_machine(self):
        state_list = DefaultStateMachine.populate()
        self.assertIsNotNone(state_list)

if __name__ == '__main__':
    unittest.main()