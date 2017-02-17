"""
Unit tests for the double timer used for the state publishing and the timeout
"""

__author__ = 'Lorenzo Vannucci, Alessandro Ambrosano'

from hbp_nrp_watchdog.Timer import Timer
import time
import threading

import unittest
import mock


class TestTimer(unittest.TestCase):

    def setUp(self):
        self.timer_return = False

    def test_timer(self):

        f = mock.Mock()

        # Testing ticks
        dt = Timer(0.1, f)
        dt.start()
        time.sleep(0.5)
        dt.cancel_all()
        self.assertGreaterEqual(f.call_count, 4)
        time.sleep(0.2)
        self.assertFalse(dt.isAlive())

if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestTimer)
    unittest.TextTestRunner(verbosity=2).run(suite)
