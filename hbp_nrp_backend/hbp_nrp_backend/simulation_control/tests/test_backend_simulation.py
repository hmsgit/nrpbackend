# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module tests the backend implementation of the simulation lifecycle
"""

from mock import Mock, patch
import unittest
import os
from mock import patch, MagicMock, mock_open
from hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle import BackendSimulationLifecycle
from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_commons.MockUtil import MockUtil
import datetime
import rospy

PATH = os.path.split(__file__)[0]

__author__ = 'Georg Hinkel'

_base_path = 'hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle.'


@patch("__builtin__.open", mock_open(read_data='somedata'))
@patch(_base_path + 'io', new=MagicMock())
@patch(_base_path + 'UserAuthentication', new=MagicMock())
class TestBackendSimulationLifecycle(unittest.TestCase):

    def setUp(self):
        self.simulation = Mock()

        self.simulation.sim_id = 42
        self.simulation.experiment_conf = "ExDXMLExample.xml"
        self.simulation.experiment_id = None
        self.simulation.state_machines = []
        self.simulation.playback_path = None
        self.simulation.private = True

        caller_id = patch("hbp_nrp_commons.simulation_lifecycle.get_caller_id", return_value="test_client")
        caller_id.start()
        self.addCleanup(caller_id.stop)

        self.rospy_mock = MockUtil.fakeit(self, _base_path + 'rospy')
        self.cle_factory_mock = MockUtil.fakeit(self, _base_path + 'ROSCLEClient')
        self.storage_mock = MockUtil.fakeit(self, _base_path + 'StorageClient')
        self.zip_util = MockUtil.fakeit(self, _base_path + 'ZipUtil')
        self.mocked_os = MockUtil.fakeit(self, _base_path + 'os')
        self.exp_mocked = MockUtil.fakeit(self, _base_path + 'exp_conf_api_gen')
        self.factory_mock = MockUtil.fakeit(self, _base_path + 'ROSCLESimulationFactoryClient')
        self.playback_mock = MockUtil.fakeit(self, 'hbp_nrp_backend.cle_interface.PlaybackClient.PlaybackClient')

        self.storage_mock.get_model.return_value = None
        self.storage_mock.get_model_path.return_value = None

        self.zip_util.extractall.return_value = None

        self.mocked_os.path.join.return_value = "/some/tmp/dir/"
        self.mocked_os.path.exists.return_value = True
        self.mocked_os.path.makedirs.return_value = None

        self.exp_mocked.CreateFromDocument.return_value.timeout.value.return_value = 1


        with patch("hbp_nrp_commons.simulation_lifecycle.Publisher"):
            with patch("hbp_nrp_commons.simulation_lifecycle.Subscriber"):
                self.lifecycle = BackendSimulationLifecycle(self.simulation)

        self.lifecycle.experiment_path = PATH
        self.assertEqual(None, self.lifecycle.sim_dir)

    def test_init_playback(self):
        self.simulation.playback_path = 'foo'
        lifecycle = BackendSimulationLifecycle(self.simulation)
        self.playback_mock.assert_callled_once_with(42)

    def test_backend_initialize_non_storage(self):
        self.lifecycle.initialize(Mock())

        # Assert state machines have been initialized
        self.assertTrue(self.simulation.state_machine_manager.add_all.called)
        self.assertTrue(self.simulation.state_machine_manager.initialize_all.called)

        # Assert Simulation server has been called
        self.assertTrue(self.factory_mock.called)

        # Assert the simulation will be killed eventually
        self.assertIsInstance(self.simulation.kill_datetime, datetime.datetime)

        self.assertIsNotNone(self.lifecycle.experiment_path)

    def test_backend_initialize_nonexisting_experiment(self):
        self.storage_mock.return_value.clone_all_experiment_files.side_effect = Exception
        self.assertRaises(NRPServicesGeneralException, self.lifecycle.initialize, Mock())

    def test_backend_initialize_noclecommunication(self):
        self.rospy_mock.ROSException = rospy.ROSException
        self.factory_mock.side_effect = rospy.ROSException

        self.assertRaises(NRPServicesGeneralException, self.lifecycle.initialize, Mock())

    def test_backend_initialize_service_problem(self):
        self.rospy_mock.ServiceException = rospy.ServiceException
        self.factory_mock.side_effect = rospy.ServiceException
        self.assertRaises(NRPServicesGeneralException, self.lifecycle.initialize, Mock())

    def test_backend_start(self):
        self.lifecycle.start(Mock())

        # Assert state machines have been started
        self.assertTrue(self.simulation.state_machine_manager.start_all.called)

    def test_backend_start_state_machines_failed(self):
        self.simulation.state_machine_manager.start_all.side_effect = IOError
        self.lifecycle.start(Mock())

        # Assert no exception, but state machine manager was still called
        self.assertTrue(self.simulation.state_machine_manager.start_all.called)

    def test_backend_stop(self):
        self.simulation.context_id = "Foobar"

        with patch('tempfile.gettempdir', return_value='foo') as mock_tempfile,\
                patch('os.path.split', return_value=['foo', 'bar']) as mock_split, \
                patch('shutil.rmtree') as mock_rmtree, \
                patch('os.listdir') as mock_list_dir, \
                patch('tempfile.mkdtemp') as mock_mkdir, \
                patch('os.makedirs') as mock_makedirs:

            self.lifecycle.stop(Mock())
            mock_tempfile.assert_called_once()
            mock_split.assert_called_once()
            mock_rmtree.assert_called_once()
            mock_list_dir.assert_called_once()
            mock_mkdir.assert_called_once()
            mock_makedirs.assert_called_once()

        # Assert State Machines have been terminated
        self.assertTrue(self.simulation.state_machine_manager.shutdown.called)
        self.assertIsNone(self.simulation.kill_datetime)

    def test_backend_pause(self):
        self.lifecycle.pause(Mock())
        # The method does nothing currently, so we have nothing to test

    def test_backend_fail(self):
        self.lifecycle.fail(Mock())

        # Assert state machines have been terminated
        self.assertTrue(self.simulation.state_machine_manager.terminate_all.called)
        self.assertIsNone(self.simulation.kill_datetime)

    def test_backend_reset(self):
        self.lifecycle.reset(Mock())

        # Assert state machines have been terminated
        self.assertTrue(self.simulation.state_machine_manager.terminate_all.called)

    def test_prepare_custom_environment_template(self):
        exp = MagicMock()
        exp.environmentModel.model = 'myAwesomeModel'
        self.lifecycle._prepare_custom_environment(exp)
        self.zip_util.extractall.assert_called_once()
