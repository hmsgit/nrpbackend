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
This module contains the REST implementation
for saving to the storage
the content of the CSV recorders of the simulation.
"""
import time
import os
import threading
from cle_ros_msgs.msg import CSVRecordedFile
from hbp_nrp_backend import get_date_and_time_string
from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient, find_file_in_paths
import hbp_nrp_cle.tf_framework as tf_framework

__author__ = 'Manos Angelidis'


class CSVLogger(object):
    """
    Class that logs CSV data to the storage,
    received from the CLE every n seconds
    Runs in a separate killable thread
    """

    def __init__(self, assembly, interval=5, folder_name=None):
        """
        The assembly object contains all the necessary information
        to save the csv data, the token, experiment_id

        :param SimulationAssembly assembly: contains all the
        information tied to the running simulation
        :param optional int interval: the interval between consequent saves
        :param optional string folder_name: user-defined name for csv_data
        """
        self._log_csv_thread = None
        self._creation_time = get_date_and_time_string()
        self._interval = interval
        self._assembly = assembly
        self._folder_name = folder_name
        self._storage_client = StorageClient()

        self.stop_flag = threading.Event()

    def initialize(self):
        """
        Initializes a killable thread which runs the log_csv function
        every interval seconds, and starts the thread
        """

        self.stop_flag.clear()

        def _log_csv_job():  # pragma: no cover
            """
            The job to put in the thread
            """
            while not self.stop_flag.isSet():
                self._log_csv()
                time.sleep(self._interval)

        self._log_csv_thread = threading.Thread(target=_log_csv_job)
        self._log_csv_thread.start()

    def shutdown(self):
        """
        Terminates the killable thread that saves the csv data
        """
        if self._log_csv_thread:
            self.stop_flag.set()
            self._log_csv()

        if self._log_csv_thread:
            self._log_csv_thread.join()  # wait till thread returns
            self._log_csv_thread = None

    def reset(self):
        """
        Resets the killable thread that saves the csv data and updates
        the creation time. This is done to create a new folder after reset
        """
        self.shutdown()
        # update the creation time to store the data in a separate folder upon reset
        self._creation_time = get_date_and_time_string()
        self.initialize()

    def _log_csv(self):
        """
        Appends the simulation CSV recorders' content to the storage
        """
        csv_files = [CSVRecordedFile(recorded_file[0], recorded_file[1], recorded_file[2])
                     for recorded_file in tf_framework.dump_csv_recorder_to_files()]
        if csv_files:
            time_string = self._creation_time if self._creation_time \
                else get_date_and_time_string()
            subfolder_name = self._folder_name if self._folder_name \
                else '_'.join(['csv_records', time_string])
            # no harm calling the function since the create_folder does
            # nothing if the folder exists
            folder_uuid = self._storage_client.create_folder(
                self._assembly.sim_config.token,
                self._assembly.sim_config.experiment_id,
                subfolder_name
            )['uuid']

            for csv_file in csv_files:
                # if there is no lock file it means that for the currently running sim
                # no csv files have been created, thus we create them in the storage and
                # append the headers. To make sure that we don't do it every time in the
                # context of the current simulation, we check if a lock file exists,
                # if not, we create it
                lock_filename = csv_file.name + '.lock'
                lock_full_path = os.path.join(
                    self._storage_client.get_simulation_directory(), subfolder_name, lock_filename)
                dirname = os.path.dirname(lock_full_path)
                lock = find_file_in_paths(lock_filename, [dirname]) or find_file_in_paths(
                    csv_file.name, [dirname])
                if not lock:
                    content = ''.join(csv_file.headers) + \
                        ''.join(csv_file.values)
                    if not os.path.exists(dirname):
                        os.makedirs(dirname)
                    with open(os.path.join(dirname, lock_filename), 'a'):
                        pass
                else:
                    content = ''.join(csv_file.values)
                self._storage_client.create_or_update(
                    self._assembly.sim_config.token, folder_uuid, csv_file.name,
                    content, 'text/plain', append=lock)
