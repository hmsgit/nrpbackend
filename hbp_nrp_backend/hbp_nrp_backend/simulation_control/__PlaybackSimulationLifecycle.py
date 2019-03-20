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
This module contains the implementation of the playback simulation lifecycle
"""

from hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle \
    import BackendSimulationLifecycle

import os
import urllib

from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient
from hbp_nrp_commons.ZipUtil import ZipUtil

from hbp_nrp_backend.__UserAuthentication import UserAuthentication
from hbp_nrp_backend import NRPServicesClientErrorException


class PlaybackSimulationLifecycle(BackendSimulationLifecycle):
    """
    This class implements the playback simulation lifecycle
    """

    def __init__(self, simulation, initial_state='created'):
        """
        Creates a new playback simulation lifecycle

        :param simulation: The simulation for which the simulation lifecycle is created
        """
        super(PlaybackSimulationLifecycle, self).__init__(simulation, initial_state)

    def initialize(self, state_change):
        """
        Initializes the simulation

        :param state_change: The state change that caused the simulation to be initialized
        """

        self.prepare_record_for_playback()
        super(PlaybackSimulationLifecycle, self).initialize(state_change)

    def start(self, state_change):
        """
        Starts the simulation playback, do not start state machines in parent
        BackendSimulationLifecycle

        :param state_change: The state change that lead to starting the simulation
        """
        pass

    def prepare_record_for_playback(self):
        """
        Copy the record from user storage to tmp and unzip it.

        :> experimentID: The experiment
        :> storagePath: The storage path

        :tmp path: The local path ready to be played
        """

        client = StorageClient()
        file_clone_destination = os.path.join(client.get_simulation_directory(),
                                              self.simulation.playback_path)  # zip path
        dest_path = os.path.dirname(file_clone_destination)

        try:
            if not os.path.exists(dest_path):
                os.makedirs(dest_path)
            with open(file_clone_destination, "w") as file_clone:

                file_contents = client.get_file(
                    UserAuthentication.get_header_token(),
                    urllib.quote_plus(self.simulation.experiment_id + '/recordings'),
                    os.path.basename(self.simulation.playback_path),  # zip name
                    by_name=True, zipped=True)

                file_clone.write(file_contents)

            ZipUtil.extractall(file_clone_destination, dest_path, True)

            # Update sim object's playback path with folder name
            self.simulation.playback_path = os.path.join(
                dest_path, ZipUtil.get_rootname(file_clone_destination))

            os.remove(file_clone_destination)

        except Exception as ex:
            client.remove_temp_sim_directory()
            raise NRPServicesClientErrorException(
                'Copying recording to backend tmp failed with {}'.format(str(ex)),
                error_code=404)
