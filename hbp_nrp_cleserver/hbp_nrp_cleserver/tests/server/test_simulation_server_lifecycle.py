"""
SimulationServerLifecycle unit test
"""

from hbp_nrp_cleserver.server.SimulationServerLifecycle import SimulationServerLifecycle
import logging
from testfixtures import log_capture
import unittest
from mock import patch, MagicMock, Mock
from hbp_nrp_cle.tf_framework import TFException


class TestSimulationServerLifecycle(unittest.TestCase):

    LOGGER_NAME = 'hbp_nrp_cleserver.server.SimulationServerLifecycle'

    def setUp(self):
        self.mock_cle = MagicMock()
        self.mock_server = MagicMock()
        self.mock_double_timer_instance = MagicMock()
        self.mock_double_timer = \
            patch('hbp_nrp_cleserver.server.SimulationServerLifecycle.DoubleTimer',
            MagicMock(return_value=self.mock_double_timer_instance))

        self.mock_event_instance = MagicMock()
        self.mock_thread_instance = MagicMock()
        self.mock_threading = \
            patch('hbp_nrp_cleserver.server.SimulationServerLifecycle.threading').start()
        self.mock_threading.Event.return_value = self.mock_event_instance
        self.mock_threading.Thread.return_value = self.mock_thread_instance

        self.ssl = SimulationServerLifecycle(0, self.mock_cle, self.mock_server)

    def test_init(self):
        self.mock_double_timer_instance.start.assert_called()
        self.mock_double_timer_instance.enable_second_callback.assert_called()
        self.mock_threading.Event.assert_called()

    def test_initialize(self):
        # Trying to initialize with initialized CLE
        self.mock_cle.is_initialized = True
        self.ssl.initialize('not relevant')
        self.assertFalse(self.mock_cle.initialize.called)
        # Trying to initialize with uninitialized CLE
        self.mock_cle.is_initialized = False
        self.ssl.initialize('not relevant')
        self.mock_cle.initialize.assert_called()

    def test_start(self):
        # Checking if starting a simulation creates a new thread
        self.ssl.start('not relevant')
        self.mock_threading.Thread.assert_called_with(
            target=self.ssl._SimulationServerLifecycle__simulation)
        self.mock_thread_instance.setDaemon.assert_called_with(True)
        self.mock_thread_instance.start.assert_called()

        self.mock_threading.Thread.call_args_list[0][1]['target']()
        self.mock_cle.start.assert_called()

        self.mock_cle.start.side_effect = TFException(None, None, None)
        self.mock_threading.Thread.call_args_list[0][1]['target']()
        self.assertEquals(self.mock_server.publish_error.call_args_list[-1][0][0],
                          "Transfer Function")

        self.mock_cle.start.side_effect = Exception()
        self.assertRaises(Exception, self.mock_threading.Thread.call_args_list[0][1]['target'])
        self.assertEquals(self.mock_server.publish_error.call_args_list[-1][0][0], "CLE")


    @log_capture(level=logging.ERROR)
    def test_stop(self, logcapture):
        # Stop transition with everything all right
        self.ssl.start('not relevant')
        self.mock_thread_instance.isAlive.return_value = False
        self.ssl.stop('not relevant')
        self.mock_double_timer_instance.cancel_all.assert_called()
        self.mock_cle.stop.assert_called()
        self.mock_thread_instance.join.assert_called_with(60)
        self.mock_double_timer_instance.join.assert_called()
        # Stop transition with a thread unwilling to stop
        self.mock_thread_instance.isAlive.return_value = True
        self.ssl.stop('not relevant')
        self.mock_double_timer_instance.cancel_all.assert_called()
        self.mock_cle.stop.assert_called()
        self.mock_thread_instance.join.assert_called_with(60)
        self.mock_double_timer_instance.join.assert_called()
        logcapture.check(
            (self.LOGGER_NAME, 'ERROR', "Error while stopping the simulation, "
                                        "impossible to join the simulation thread")
        )

    def test_shutdown(self):
        # Testing shutdown
        self.ssl.shutdown('not relevant')
        self.mock_event_instance.set.assert_called()

    def test_fail(self):
        # Testing fail
        self.ssl.fail('not relevant')
        self.mock_cle.stop.assert_called()
        self.mock_double_timer_instance.cancel_all.assert_called()

    def test_pause(self):
        self.ssl.pause('not relevant')
        self.mock_cle.stop.assert_called()

    def test_reset(self):
        self.ssl.reset('not relevant')
        self.mock_server.start_fetching_gazebo_logs.assert_called()
        self.mock_cle.stop.assert_called()
        self.mock_cle.reset.assert_called()
        self.mock_server.stop_fetching_gazebo_logs.assert_called()

    def test_done_event(self):
        self.assertEquals(self.ssl.done_event, self.mock_event_instance)


if __name__ == '__main__':
    unittest.main()
