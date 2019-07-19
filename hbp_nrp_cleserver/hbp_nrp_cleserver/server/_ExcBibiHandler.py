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
This module provides support methods to perform I/O operation of exc and bibi files.
"""
import os
import logging
from hbp_nrp_commons.sim_config.ConfigEditor import ConfigEditor

logger = logging.getLogger(__name__)


# This class is being intentionally excluded from code coverage
class ExcBibiHandler(object):   # pragma: no cover
    """
    Helper class for ROSCLEServer to handle read write operations on exc and bibi in Storage
    """

    def __init__(self, assembly):
        self._cle_assembly = assembly
        self._conf_editor = ConfigEditor(assembly.sim_config)

    def add_robotpose(self, robot_id, pose=None):
        """
        Adds a <robotPose> tag in the exc

        :param robot_id: robotId attribute for the tag
        :param pose: A cle_ros_msgs.msgs.Pose object (defines an object's Euler pos and orientation)
        :return:
        """

        self._conf_editor.add_robotpose(robot_id, pose)
        self._write_file_to_storage(
            self._cle_assembly.sim_config.exc_abs_path,
            os.path.basename(self._cle_assembly.sim_config.exc_abs_path))

    def delete_robotpose(self, robot_id):
        """
        Edit <robotPose> tag in the exc where robotId=robot_id

        :param robot_id: robotId attribute for the tag
        :return: Tuple (True, SDF relative path) or (False, error message) to update config files
        """

        self._conf_editor.delete_robotpose(robot_id)
        self._write_file_to_storage(
            self._cle_assembly.sim_config.exc_abs_path,
            os.path.basename(self._cle_assembly.sim_config.exc_abs_path))

    def update_robotpose(self, robot_id, pose):
        """
        Edit <robotPose> tag in the exc where robotId=robot_id

        :param robot_id: robotId attribute for the tag
        :param pose: A cle_ros_msgs.msgs.Pose object (defines an object's Euler pos and orientation)
        :return: Tuple (True, SDF relative path) or (False, error message) to update config files
        """

        ret, status = self._conf_editor.update_robotpose(robot_id, pose)
        if ret:
            self._write_file_to_storage(
                self._cle_assembly.sim_config.exc_abs_path,
                os.path.basename(self._cle_assembly.sim_config.exc_abs_path))

        return ret, status

    def add_bodymodel(self, robot_id, model_path, is_custom, robot_model=None):
        """
        Adds a <bodyModel> tag in the bibi

        :param robot_id: attribute robotId in the tag
        :param model_path: value() fo the tag
        :param is_custom: attribute isCustom in the tag
        :param robot_model: if custom, name of the custom template model
        """

        self._conf_editor.add_bodymodel(robot_id, model_path, is_custom, robot_model)
        self._write_file_to_storage(
            self._cle_assembly.sim_config.bibi_path.abs_path,
            self._cle_assembly.sim_config.bibi_path.rel_path)

    def delete_bodymodel(self, robot_id):
        """
        Deletes a <bodyModel> tag from the bibi

        :param robot_id: attribute robotId in the tag
        """

        self._conf_editor.delete_bodymodel(robot_id)
        self._write_file_to_storage(
            self._cle_assembly.sim_config.bibi_path.abs_path,
            self._cle_assembly.sim_config.bibi_path.rel_path)

    def _write_file_to_storage(self, file_abs_path, in_storage_rel_path):
        """
        Write a file from the backend to the storage

        :param file_abs_path: absolute path in the backend
        :param in_storage_rel_path: location in the storage to write
        """

        # update storage's copy
        with open(file_abs_path) as _file:
            self._cle_assembly.storage_client.create_or_update(
                self._cle_assembly.sim_config.token,
                self._cle_assembly.sim_config.experiment_id,
                in_storage_rel_path,
                _file.read(),
                "text/plain"
            )
