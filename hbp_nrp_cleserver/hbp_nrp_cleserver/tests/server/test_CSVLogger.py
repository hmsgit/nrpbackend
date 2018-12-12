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
Unit tests for the simulation csv logger
"""
__author__ = 'Manos Angelidis'

import unittest
from mock import patch, MagicMock, Mock
from hbp_nrp_cleserver.server.CSVLogger import CSVLogger


class MockAssembly(object):
    def __init__(self):
        self.token = 'token'
        self.experiment_id = 'expId'


class MockKillable(object):
    def __init__(self):
        return

    def terminate(self):
        return

    def start(self):
        return


class CSVRecordedMock(object):
    def __init__(self, name, headers, values):
        self.name = name
        self.headers = headers
        self.values = values


class MockTFFramework(object):
    def dump_csv_recorder_to_files(self):
        return [MagicMock(name='test.csv',
                          headers=['header1', 'header2\n'],
                          values=['recorded_data1', 'recorded_data2\n'])]


class TestSimulationCSVLogger(unittest.TestCase):
    def setUp(self):
        patch_StorageClient = patch('hbp_nrp_cleserver.server.CSVLogger.StorageClient')
        self.addCleanup(patch_StorageClient.stop)
        self.mock_StorageClient = patch_StorageClient.start()
        self.mock_storageClient_instance = self.mock_StorageClient.return_value
        patch_recorded_mock = patch('hbp_nrp_cleserver.server.CSVLogger.CSVRecordedFile')
        self.addCleanup(patch_recorded_mock.stop)
        self.mock_recorded = patch_recorded_mock.start()

    @patch('hbp_nrp_cleserver.server.CSVLogger.get_date_and_time_string')
    def test_CSV_logger_init(self, mock_get_date_and_time_string):
        mock_get_date_and_time_string.return_value = 'fakeTime'
        csv_logger = CSVLogger(MockAssembly(), 5, 'testFolder')
        self.assertEqual(csv_logger._creation_time, 'fakeTime')
        self.assertEqual(csv_logger._folder_name, 'testFolder')
        self.assertEqual(csv_logger._interval, 5)
        self.assertEqual(csv_logger._log_csv_thread, None)
        self.assertEqual(csv_logger._assembly.token, 'token')
        self.assertEqual(csv_logger._assembly.experiment_id, 'expId')

    @patch('hbp_nrp_cleserver.server.CSVLogger.killable_threads.Thread')
    @patch('hbp_nrp_cleserver.server.CSVLogger.get_date_and_time_string')
    def test_CSV_logger_lifecycle(self, mock_get_date_and_time_string, mock_killable):
        mock_get_date_and_time_string.return_value = 'fakeTime'
        mock_killable = MockKillable()
        csv_logger = CSVLogger(MockAssembly(), 5, 'testFolder')
        csv_logger.initialize()
        self.assertIsInstance(csv_logger._log_csv_thread, MagicMock)
        self.assertTrue(csv_logger._log_csv_thread.start.called)
        csv_logger.shutdown()
        self.assertTrue(csv_logger._log_csv_thread.terminate.called)
        mock_get_date_and_time_string.return_value = 'newTime'
        csv_logger.reset()
        self.assertEqual(csv_logger._creation_time, 'newTime')

    @patch('__builtin__.open')
    @patch('os.makedirs')
    @patch('os.path.exists')
    @patch("hbp_nrp_cleserver.server.CSVLogger.find_file_in_paths")
    @patch('hbp_nrp_cleserver.server.CSVLogger.killable_threads.Thread')
    @patch('hbp_nrp_cleserver.server.CSVLogger.tf_framework')
    @patch('hbp_nrp_cleserver.server.CSVLogger.get_date_and_time_string')
    def test_CSV_logger_log(self, mock_get_date_and_time_string, mock_tf_framework,
                            mock_killable, mock_find_file, mock_os_exists, mock_os_makedirs,
                            mock_open):
        time = 'fakeTime'
        mock_get_date_and_time_string.return_value = time
        mock_tf_framework.dump_csv_recorder_to_files = MockTFFramework().dump_csv_recorder_to_files
        mock_killable = MockKillable()
        mock_os_exists.return_value = False
        self.mock_recorded.return_value = CSVRecordedMock("bar1", ["bar1 header\n"], ['data1', 'data2\n'])
        csv_logger = CSVLogger(MockAssembly(), 5, 'testFolder')
        self.mock_storageClient_instance.create_or_update.side_effect = None
        self.mock_storageClient_instance.create_folder.return_value = {"uuid": "mockUUID"}
        csv_logger.initialize()
        csv_logger._log_csv()
        self.mock_storageClient_instance.create_or_update.assert_called_once()
        self.mock_storageClient_instance.create_folder.assert_called_once_with('token', 'expId', 'testFolder')
        mock_find_file.return_value = False
        csv_logger._log_csv()
        self.mock_storageClient_instance.create_or_update.assert_called_with(
            'token', 'mockUUID', 'bar1', 'bar1 header\ndata1data2\n', 'text/plain', append=False)
        self.mock_storageClient_instance.create_folder.assert_called_with('token', 'expId', 'testFolder')
        mock_find_file.assert_called_once()
        mock_find_file.return_value = True
        csv_logger._log_csv()
        self.mock_storageClient_instance.create_or_update.assert_called_with(
            'token', 'mockUUID', 'bar1', 'data1data2\n', 'text/plain', append=True)
