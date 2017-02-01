from hbp_nrp_cleserver.server.LocalGazebo import LocalGazeboServerInstance, LocalGazeboBridgeInstance
from hbp_nrp_cle import config
import unittest
from mock import patch, MagicMock

__author__ = 'Alessandro Ambrosano, Georg Hinkel'


class TestLocalGazeboServerInstance(unittest.TestCase):

    def setUp(self):
        self.instance = LocalGazeboServerInstance()
        self.has_died = False
        self.die_error = None

    def died_callback(self):
        self.has_died = True

    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.Watchdog')
    def test_start(self, mocked_watchdog, mocked_os):
        self.instance.start('')
        mocked_os.system.assert_any_call(config.config.get('gazebo', 'restart-cmd'))
        self.assertTrue(mocked_watchdog.called)
        self.assertTrue(mocked_watchdog().start.called)

    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.Watchdog')
    def test_stop(self, mocked_watchdog, mocked_os):
        self.instance.start('')
        self.instance.stop()
        mocked_os.system.assert_any_call(config.config.get('gazebo', 'stop-cmd'))
        self.assertTrue(mocked_watchdog().stop.called)

    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.Watchdog')
    def test_restart(self, mocked_watchdog, mocked_os):
        self.instance.restart('')
        mocked_os.system.assert_any_call(config.config.get('gazebo', 'restart-cmd'))
        self.assertTrue(mocked_watchdog.called)
        self.assertTrue(mocked_watchdog().start.called)

    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    def test_gazebo_master_uri(self, mocked_os):
        mocked_os.environ.get = MagicMock(return_value=None)
        self.assertIsInstance(self.instance.gazebo_master_uri, str)
        self.assertNotEqual(self.instance.gazebo_master_uri, "")
        mock_gazebo_master_uri = "http://localhost:12345"
        mocked_os.environ.get = MagicMock(return_value=mock_gazebo_master_uri)
        self.assertIsInstance(self.instance.gazebo_master_uri, str)
        self.assertEquals(self.instance.gazebo_master_uri, mock_gazebo_master_uri)

    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.Watchdog')
    def test_raises_callback(self, mocked_watchdog, mocked_os):
        self.instance.start('')
        self.instance.gazebo_died_callback = self.died_callback
        callback = mocked_watchdog.call_args[0][1]
        callback()
        self.assertTrue(self.has_died)

    def test_can_extend(self):
        self.assertTrue(self.instance.try_extend("does not matter"))


class TestLocalGazeboBridgeInstance(unittest.TestCase):
    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.config')
    def test_start_performs_command(self, mocked_config, mocked_os):
        mocked_config.config.get.return_value = "foo"
        bridge = LocalGazeboBridgeInstance()
        bridge.start()
        mocked_os.system.assert_called_once_with("foo")
        mocked_config.config.get.assert_called_once_with('gzbridge', 'start_cmd')

    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.config')
    def test_stop_performs_command(self, mocked_config, mocked_os):
        mocked_config.config.get.return_value = "bar"
        bridge = LocalGazeboBridgeInstance()
        bridge.stop()
        mocked_os.system.assert_called_once_with("bar")
        mocked_config.config.get.assert_called_once_with('gzbridge', 'stop_cmd')


    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.config')
    def test_restart_performs_command(self, mocked_config, mocked_os):
        mocked_config.config.get.return_value = "foobar"
        bridge = LocalGazeboBridgeInstance()
        bridge.restart()
        mocked_os.system.assert_called_once_with("foobar")
        mocked_config.config.get.assert_called_once_with('gzbridge', 'restart_cmd')


if __name__ == '__main__':
    unittest.main()
