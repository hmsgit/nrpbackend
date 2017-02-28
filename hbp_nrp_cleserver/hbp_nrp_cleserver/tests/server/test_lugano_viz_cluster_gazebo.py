from hbp_nrp_cleserver.server.LuganoVizClusterGazebo import LuganoVizClusterGazebo

import unittest
from mock import patch, Mock, MagicMock
import os

__author__ = 'Alessandro Ambrosano'


class TestLuganoVizClusterGazebo(unittest.TestCase):

    def setUp(self):
        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_spawn_ssh_slurm_frontend(self, mock_spawn):
        mock_spawn().sendline = Mock()

        # Successful call
        mock_spawn().expect = Mock(return_value=0)
        mock_spawn.call_count = 0
        self.instance._LuganoVizClusterGazebo__spawn_ssh_SLURM_frontend()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline, 0)

        # Missing password
        mock_spawn().expect = Mock(return_value=1)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_ssh_SLURM_frontend)

        # Timeout
        mock_spawn().expect = Mock(return_value=2)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_ssh_SLURM_frontend)

    @patch('pexpect.spawn')
    def test_allocate_job(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__spawn_ssh_SLURM_frontend = mock_spawn
        mock_spawn().sendline = Mock()

        # Successful call
        mock_spawn().expect = Mock(return_value=1)
        mock_spawn().after = 'JobState=RUNNING'
        mock_spawn.call_count = 0
        self.instance._LuganoVizClusterGazebo__allocate_job()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)

        # Job not running
        mock_spawn().expect = Mock(return_value=1)
        mock_spawn().after = 'JobState=WRONG'
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__allocate_job)

        # Kerberos authentication missing
        mock_spawn().expect = Mock(return_value=2)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__allocate_job)

        # General error
        mock_spawn().expect = Mock(return_value=3)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__allocate_job)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_deallocate_job(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()
        mock_spawn().sendline = Mock()

        self.instance._LuganoVizClusterGazebo__deallocate_job()
        self.assertTrue(True in ['exit' in x for x in [str(y) for y in mock_spawn().sendline.mock_calls]])
        self.assertEqual('UNDEFINED', self.instance._LuganoVizClusterGazebo__state)
        self.assertIsNone(self.instance._LuganoVizClusterGazebo__allocation_process)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_start_fake_x(self, mock_spawn):
        mock_spawn().expect = Mock(return_value=1)
        mock_spawn.call_count = 0

        self.instance._LuganoVizClusterGazebo__start_fake_X()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().expect.call_count, 0)

        mock_spawn().expect = Mock(return_value = 3)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_fake_X)

    @patch('pexpect.spawn')
    def test_spawn_vglconnect(self, mock_spawn):
        # failure: __node is None
        self.instance._LuganoVizClusterGazebo__node = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_vglconnect)

        # failure: __allocation_process is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__allocation_process = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_vglconnect)

        self.instance._LuganoVizClusterGazebo__node = 'something that won\'t be used'
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()

        mock_spawn().sendline = Mock()
        mock_spawn().expect = Mock(return_value=0)
        mock_spawn.call_count = 0

        # successful call
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)

        # password missing
        mock_spawn().expect = Mock(return_value=1)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_vglconnect)

        # generic error
        mock_spawn().expect = Mock(return_value=2)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_vglconnect)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_clean_remote_files(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect = mock_spawn
        mock_spawn().sendline = Mock()

        # successful call
        self.instance._LuganoVizClusterGazebo__node = 'something that won\'t be used'
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()

        mock_spawn.call_count = 0
        self.instance._LuganoVizClusterGazebo__clean_remote_files()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_start_xvnc(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect = mock_spawn
        mock_spawn().sendline = Mock()

        # failure: __node is None
        self.instance._LuganoVizClusterGazebo__node = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_xvnc)

        # failure: __allocation_process is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__allocation_process = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_xvnc)

        self.instance._LuganoVizClusterGazebo__node = 'something that won\'t be used'
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()

        mock_spawn().sendline = Mock()

        # error creating vnc
        mock_spawn().expect = Mock(return_value=1)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_xvnc)

        # successful call
        mock_spawn().expect = Mock(return_value=0)
        mock_spawn.call_count = 0
        self.instance._LuganoVizClusterGazebo__start_xvnc()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)
        self.assertNotEqual(mock_spawn().expect.call_count, 0)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    @patch('hbp_nrp_cleserver.server.LuganoVizClusterGazebo.os.environ')
    def test_start_gazebo(self, mock_os_environ, mock_spawn):
        # failure: __node is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__node = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        # failure: __allocation_process is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__allocation_process = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        # failure: __remote_display_port is -1
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__remote_display_port = -1
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__node = 'not none'
        self.instance._LuganoVizClusterGazebo__allocation_process = 'this neither'
        self.instance._LuganoVizClusterGazebo__remote_display_port = 1
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect = mock_spawn

        mock_spawn().sendline = Mock()
        mock_spawn().expect = Mock(return_value=1)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        # successful call
        mock_spawn().expect = Mock(return_value=0)
        mock_spawn().sendline.reset_mock()
        mock_spawn.reset_mock()
        nrp_variables_path = os.path.join(os.path.dirname(__file__), 'nrp-variables')
        mock_os_environ.get = Mock(side_effect=['staging', nrp_variables_path])

        self.instance._LuganoVizClusterGazebo__start_gazebo('random_master_uri')
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertEqual(mock_os_environ.get.call_count, 2)
        mock_spawn().sendline.assert_any_call('export ENVIRONMENT=staging')
        mock_spawn().sendline.assert_any_call('ADDITIONAL_PACKAGE_VERSION=1.1.0')
        mock_spawn().sendline.assert_any_call('SMALL_PACKAGE_VERSION=2.5.5')
        self.assertGreater(mock_spawn().sendline.call_count, 3)
        self.assertNotEqual(mock_spawn().expect, 0)

        self.instance = LuganoVizClusterGazebo()

    @patch('subprocess.call')
    @patch('pexpect.spawn')
    @patch('hbp_nrp_cleserver.server.LuganoVizClusterGazebo.os.environ')
    def test_models_path(self, mock_os_environ, mock_spawn, mock_system):
        mock_system.return_value = 0
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__node = 'not none'
        self.instance._LuganoVizClusterGazebo__allocation_process = 'this neither'
        self.instance._LuganoVizClusterGazebo__remote_display_port = 1
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect = mock_spawn

        mock_spawn().expect.return_value = 0
        mock_spawn().readline.return_value = '/somewhere/under/the/rainbow'
        mock_spawn.call_count = 0
        nrp_variables_path = os.path.join(os.path.dirname(__file__), 'nrp-variables')
        mock_os_environ.get = Mock(side_effect=['staging', nrp_variables_path])

        self.instance._LuganoVizClusterGazebo__start_gazebo('random_master_uri', '/somewhere/over/the/rainbow')
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)
        self.assertNotEqual(mock_spawn().expect.call_count, 0)
        self.assertEqual(mock_spawn().readline.call_count, 2)

        mock_system.assert_any_call('scp -r /somewhere/over/the/rainbow bbpnrsoa@bbpviz1.cscs.ch:/somewhere/under/the/rainbow')

        # check error cases
        with self.assertRaises(Exception):
            mock_system.return_value = 9
            self.instance._LuganoVizClusterGazebo__start_gazebo('random_master_uri',
                                                                '/somewhere/over/the/rainbow')
        with self.assertRaises(Exception):
            mock_system.return_value = 10
            self.instance._LuganoVizClusterGazebo__start_gazebo('random_master_uri',
                                                                '/somewhere/over/the/rainbow')
        with self.assertRaises(Exception):
            mock_system.return_value = 65
            self.instance._LuganoVizClusterGazebo__start_gazebo('random_master_uri',
                                                                '/somewhere/over/the/rainbow')
        with self.assertRaises(Exception):
            mock_system.return_value = 66
            self.instance._LuganoVizClusterGazebo__start_gazebo('random_master_uri',
                                                                '/somewhere/over/the/rainbow')
        with self.assertRaises(Exception):
            mock_system.return_value = 42
            self.instance._LuganoVizClusterGazebo__start_gazebo('random_master_uri',
                                                                '/somewhere/over/the/rainbow')

        self.instance = LuganoVizClusterGazebo()

    def test_start(self):
        self.instance._LuganoVizClusterGazebo__allocate_job = Mock()
        self.instance._LuganoVizClusterGazebo__start_fake_X = Mock()
        self.instance._LuganoVizClusterGazebo__start_xvnc = Mock()
        self.instance._LuganoVizClusterGazebo__start_gazebo = Mock()

        self.instance.start('random_master_uri')
        self.assertEqual(self.instance._LuganoVizClusterGazebo__allocate_job.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__start_fake_X.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__start_xvnc.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__start_gazebo.call_count, 1)
        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_stop(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__clean_remote_files = Mock()
        self.instance._LuganoVizClusterGazebo__deallocate_job = Mock()

        self.instance._LuganoVizClusterGazebo__remote_xvnc_process = Mock()
        self.instance._LuganoVizClusterGazebo__gazebo_remote_process = Mock()
        self.instance._LuganoVizClusterGazebo__x_server_process = Mock()

        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()
        mock_spawn().sendline = Mock()

        self.instance.stop()
        self.assertEqual(self.instance._LuganoVizClusterGazebo__clean_remote_files.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__deallocate_job.call_count, 1)

        self.instance = LuganoVizClusterGazebo()

    def test_restart(self):
        with patch("hbp_nrp_cleserver.server.LuganoVizClusterGazebo.LuganoVizClusterGazebo.start"):
            with patch("hbp_nrp_cleserver.server.LuganoVizClusterGazebo.LuganoVizClusterGazebo.stop"):
                self.instance.restart('')
                self.assertEqual(self.instance.start.call_count, 1)
                self.assertEqual(self.instance.stop.call_count, 1)

if __name__ == '__main__':
    unittest.main()
