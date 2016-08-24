"""
Unit tests for the double timer used for the state publishing and the timeout
"""

__author__ = 'Lorenzo Vannucci, Alessandro Ambrosano'

from hbp_nrp_cleserver.server.DoubleTimer import DoubleTimer
import time
import threading

import unittest
import mock


class TestDoubleTimer(unittest.TestCase):

    def setUp(self):
        self.timer_return = False

    def test_timer(self):
        # Wrong initializations
        self.assertRaises(ValueError, DoubleTimer, -1, None, 1, None)
        self.assertRaises(ValueError, DoubleTimer, 1, None, -1, None)
        self.assertRaises(ValueError, DoubleTimer, 1.5, None, 7, None)

        f1 = mock.Mock()
        f2 = mock.Mock()

        # Testing ticks
        dt = DoubleTimer(0.1, f1, 3.0, f2)
        dt.enable_second_callback()
        self.assertEqual(dt.remaining_time(), 3.0)
        dt.start()
        time.sleep(0.5)
        self.assertTrue(dt.remaining_time() < 3.0)
        dt.cancel_all()

        # Testing counting reset
        dt = DoubleTimer(0.1, f1, 3.0, f2)
        dt.enable_second_callback()
        dt.start()
        time.sleep(0.5)
        self.assertTrue(dt.remaining_time() < 3.0)
        dt.disable_second_callback()
        self.assertEqual(dt.remaining_time(), 3.0)
        dt.cancel_all()

        # Testing everything is called the right number of times
        f1.call_count = 0
        f2.call_count = 0
        dt = DoubleTimer(0.05, f1, 0.25, f2)
        dt.enable_second_callback()
        dt.start()
        time.sleep(0.5)
        self.assertTrue(f1.call_count > 2)
        self.assertEqual(f2.call_count, 1)
        dt.cancel_all()


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestDoubleTimer)
    unittest.TextTestRunner(verbosity=2).run(suite)
