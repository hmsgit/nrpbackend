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
This module represents the configuration of a running simulation
"""

import os
import logging

__author__ = 'Hossain Mahmud'

logger = logging.getLogger(__name__)


class _Settings(object):
    """
    Settings that are common on the backend process or machine
    This class is a Singleton (per process)
    """

    __instance = None

    def __new__(cls):
        """
        Overridden new for the singleton implementation
        :return: Singleton instance
        """

        if _Settings.__instance is None:
            _Settings.__instance = object.__new__(cls)
        return _Settings.__instance

    def __init__(self):
        """
        Declare all the config and system variables
        """

        self.nrp_home = os.environ.get('HBP', None)
        if self.nrp_home is None:
            raise Exception("Please export NRP home directory as 'HBP' environment variable")

        self.nrp_models_directory = os.environ.get('NRP_MODELS_DIRECTORY')

        self.sim_dir_symlink = os.environ.get('NRP_SIMULATION_DIR')
        if self.sim_dir_symlink is None:
            raise Exception(
                "Simulation directory symlink location is not specified in NRP_SIMULATION_DIR")

        self.local_gazebo_path = [
            os.path.join(os.environ['HOME'], '.local', 'share', 'gazebo-7', 'media')
        ]
        self.gzweb_assets_media_path = [
            os.path.join(self.nrp_home, 'gzweb', 'http', 'client', 'assets', 'media')
        ]
        self.gzweb_custom_textures_path = [
            os.path.join(self.nrp_home, 'gzweb', 'http', 'client', 'assets', 'custom_textures')
        ]

        self.storage_uri = 'http://localhost:9000/storage'

        self.MAX_SIMULATION_TIMEOUT = 24 * 60 * 60   # seconds


# Instantiate the singleton
Settings = _Settings()
