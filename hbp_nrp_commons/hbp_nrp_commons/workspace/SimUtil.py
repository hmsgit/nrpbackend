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
This module implements Utility functions used backend
"""

import os
import errno
import shutil
import logging
import tempfile

from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_commons.workspace.Settings import Settings

__author__ = 'Hossain Mahmud'

logger = logging.getLogger(__name__)


class SimUtil(object):
    """
    Utility methods for a simulation
    """

    @staticmethod
    def makedirs(directory):
        """
        Creates [nested] directory if not exists
        :raises all error except directory exists
        """
        try:
            os.makedirs(directory)
        except OSError as e:
            if e.errno != errno.EEXIST:
                logger.exception('An error happened trying to create ' + directory)
                raise

    @staticmethod
    def mkdir(directory):
        """
        Creates [nested] directory if not exists
        :raises all error except directory exists
        """
        SimUtil.makedirs(directory)

    @staticmethod
    def rmdir(directory):
        """
        Removes [nested] directory if not exists
        :raises all error except directory exists
        """

        shutil.rmtree(os.path.realpath(directory))

    @staticmethod
    def clear_dir(directory):
        """
        Deletes contents of a directory
        """

        shutil.rmtree(os.path.realpath(directory))
        SimUtil.makedirs(directory)

    @staticmethod
    def init_simulation_dir():
        """
        Creates a temporary directory and links it to Settings.sim_dir_symlink
        """

        sim_dir = tempfile.mkdtemp(prefix='nrp.')
        try:
            if os.path.exists(Settings.sim_dir_symlink):
                shutil.rmtree(os.path.realpath(Settings.sim_dir_symlink))
                os.unlink(Settings.sim_dir_symlink)

            os.symlink(sim_dir, Settings.sim_dir_symlink)
        except IOError as err:
            raise NRPServicesGeneralException(
                "Could not create symlink to temp simulation folder. {err}".format(err=err))
        except OSError as ex:
            raise NRPServicesGeneralException(
                "Could not create symlink to temp simulation folder. {err}".format(err=ex))

        return sim_dir

    @staticmethod
    def delete_simulation_dir():
        """
        Removes simulation directory
        """

        try:
            if os.path.exists(Settings.sim_dir_symlink):
                shutil.rmtree(os.path.realpath(Settings.sim_dir_symlink))
                os.unlink(Settings.sim_dir_symlink)
        except IOError as err:
            raise NRPServicesGeneralException(
                "Could not create symlink to temp simulation folder. {err}".format(err=err))
        except OSError as ex:
            raise NRPServicesGeneralException(
                "Could not create symlink to temp simulation folder. {err}".format(err=ex))

    @staticmethod
    def find_file_in_paths(file_rel_path, path_list):
        """
        :return: returns the absolute path of the first file found path_list.
                 if not found returns and empty string.
        """

        for file_path in (p for p in path_list if os.path.isfile(os.path.join(p, file_rel_path))):
            return os.path.join(file_path, file_rel_path)

        return None
