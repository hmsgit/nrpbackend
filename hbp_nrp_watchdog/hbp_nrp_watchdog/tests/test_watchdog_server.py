"""
This module tests the watchdog server and client implementation
"""

import unittest
from hbp_nrp_watchdog.WatchdogServer import WatchdogServer, WatchdogClient
from mock import patch, Mock

__author__ = "Georg Hinkel"


class TestWatchdogServer(unittest.TestCase):

    @patch("hbp_nrp_watchdog.WatchdogServer.rospy")
    def setUp(self, rospy_mock):
        with patch("hbp_nrp_watchdog.Watchdog.Timer") as timer_mock:
            self.watchdog = WatchdogServer("gzserver", "/foo")
            self.assertEqual("/foo", rospy_mock.Publisher.call_args[0][0])
            self.timer_mock = timer_mock()
        self.publisher = rospy_mock.Publisher()

    def test_watchdog_server_publishes_alive_signal(self):
        is_alive = Mock()
        self.watchdog._is_alive = is_alive

        is_alive.return_value = True
        self.watchdog._watch()
        self.publisher.publish.assert_called_once_with(True)
        self.publisher.publish.reset_mock()

        is_alive.return_value = False
        self.watchdog._watch()
        self.publisher.publish.assert_called_once_with(False)

    def test_stop_unregisters_publisher(self):
        self.watchdog.stop()
        self.publisher.unregister.assert_called_once_with()
        self.timer_mock.cancel_all.assert_called_once_with()

class TestWatchdogClient(unittest.TestCase):

    def __callback(self):
        self.__callback_called = True

    @patch("hbp_nrp_watchdog.WatchdogServer.rospy")
    def setUp(self, rospy_mock):
        self.__callback_called = False
        with patch("hbp_nrp_watchdog.WatchdogServer.Timer") as timer_mock:
            self.watchdog = WatchdogClient("/foo", self.__callback)
            self.assertEqual("/foo", rospy_mock.Subscriber.call_args[0][0])
            self.subscriber_callback = rospy_mock.Subscriber.call_args[0][2]
            self.timer_callback = timer_mock.call_args[0][1]
            self.timer_mock = timer_mock()
        self.subscriber = rospy_mock.Subscriber()

    def test_start_starts_timer(self):
        self.watchdog.start()
        self.timer_mock.start.assert_called_once_with()

    def test_stop_stops_timer_and_unregisters(self):
        self.watchdog.stop()
        self.timer_mock.cancel_all.assert_called_once_with()
        self.subscriber.unregister.assert_called_once_with()

    def test_subscriber_callback(self):
        data = Mock()
        data.data = True
        self.subscriber_callback(data)
        self.assertFalse(self.__callback_called)
        data.data = False
        self.subscriber_callback(data)
        self.assertTrue(self.__callback_called)

    @patch("hbp_nrp_watchdog.WatchdogServer.time")
    def test_timer_callback(self, time_mock):
        time_mock.time.return_value = 42
        data = Mock()
        data.data = True
        self.subscriber_callback(data)
        time_mock.time.return_value = 43
        self.timer_callback()
        self.assertFalse(self.__callback_called)
        time_mock.time.return_value = 44
        self.timer_callback()
        self.assertFalse(self.__callback_called)
        time_mock.time.return_value = 47.001
        self.timer_callback()
        self.assertTrue(self.__callback_called)
