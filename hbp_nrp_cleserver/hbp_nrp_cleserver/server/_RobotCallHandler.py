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
This module provides support methods to perform operation using cle_nrp_cle.RobotManager
"""

__author__ = 'Hossain Mahmud'

import os
import logging

from pyxb import ValidationError

from cle_ros_msgs import msg
from hbp_nrp_cle.robotsim.RobotManager import Robot
from hbp_nrp_commons.generated import robot_conf_api_gen as robotXmlParser

logger = logging.getLogger(__name__)


class RobotCallHandler(object):
    """
    Helper class for ROSCLEServer to handle robot operations
    """
    def __init__(self, assembly):
        self._cle_assembly = assembly

    def get_robots(self):
        """
        Gets robots list in the currently running simulation
        REST request GET /robots/<sim_id> ends up here

        :return: A list containing rospy msg object of the robot list
        """
        ret = []
        for rid, robot in self._cle_assembly.robotManager.get_robot_list().iteritems():
            ret.append(msg.RobotInfo(str(rid), str(robot.SDFFileRelPath), bool(robot.isCustom)))

        return ret

    def add_robot(self, robot_id, robot_model_rel_path, is_custom=False, pose=None):
        """
        Adds a robot in the currently running simulation
        REST request PUT /robots/<sim_id> ends up here

        :param robot_id: Id of the robot
        :param robot_model_rel_path: SDF or the zip path of the robot
        :param is_custom: is the asset custom (zip or sdf !)
        :param pose: initial robot pose
        :return: Tuple (True, SDF relative path) or (False, error message) to update config files
        """
        # pylint: disable=too-many-locals,too-many-branches,too-many-statements
        from hbp_nrp_backend.storage_client_api.StorageClient import \
            find_file_in_paths, get_model_basepath
        client = self._cle_assembly.storage_client
        simDir = self._cle_assembly.simdir

        robot_sdf_abs_path = None
        try:
            if is_custom:
                zipRelPath = robot_model_rel_path
                import json
                try:
                    # Hack the file name for the storage server. Replace spaces and slashes.
                    # How do i know this? devine knowledge!
                    requestFile = ('robots/' + zipRelPath).replace(' ', '%20').replace('/', '%2F')
                    # download zip from storage
                    data = client.get_custom_model(
                        self._cle_assembly.token,
                        self._cle_assembly.ctx_id,
                        json.dumps({'uuid': requestFile})
                    )
                    # write in the current simulation directory
                    if not os.path.exists(os.path.join(simDir, os.path.dirname(zipRelPath))):
                        os.makedirs(os.path.join(simDir, os.path.dirname(zipRelPath)))
                    with open(os.path.join(simDir, zipRelPath), 'w') as destAbsPath:
                        destAbsPath.write(data)

                    # extract
                    if not os.path.exists(os.path.join(simDir, self._cle_assembly.tempAssetsDir)):
                        os.makedirs(os.path.join(simDir, self._cle_assembly.tempAssetsDir))
                    import zipfile
                    with zipfile.ZipFile(os.path.join(simDir, zipRelPath)) as rzip:
                        rzip.extractall(os.path.join(simDir, self._cle_assembly.tempAssetsDir))

                    # get the root directory within the zip
                    rootfolder = rzip.namelist()[0]

                    # Get robot sdf name from model.config
                    with open(os.path.join(simDir, self._cle_assembly.tempAssetsDir,
                                           rootfolder, 'model.config')) as conf:
                        try:
                            confDOM = robotXmlParser.CreateFromDocument(conf.read())
                        except ValidationError as ve:
                            raise Exception("Could not parse experiment configuration {0:s} due "
                                            "to validation error: {1:s}".format(conf, str(ve)))

                    if confDOM.sdf is None:
                        return False, "'model.config' has no 'sdf' tag"

                    sdf_filename = confDOM.sdf.value()
                    sdf_abs_path = os.path.join(simDir, 'assets', rootfolder, sdf_filename)

                    if not os.path.isfile(sdf_abs_path):
                        return False, "No SDF named {name} found at {loc} specified " \
                                      "in model.config in the uploaded zip" \
                            .format(name=confDOM.sdf.value(), loc=rootfolder)

                # pylint: disable=broad-except
                except Exception as e:
                    return False, str(e)
                # TODO: ask proxy to copy sdf into experiment folder!

            else:   # if template robot
                # find the SDF in the template directories
                sdf_abs_path = find_file_in_paths(robot_model_rel_path, get_model_basepath())
                if not sdf_abs_path:
                    raise Exception("Could not find {0} in the template library"
                                    .format(robot_model_rel_path))
                sdf_filename = os.path.basename(sdf_abs_path)

            if not pose:
                # HACK: find first robot, and use its pose
                if len(self._cle_assembly.robotManager.get_robot_list()):
                    r = self._cle_assembly.robotManager.get_robot_list().values()[0]
                    pose = r.pose

            # copy sdf to <simulation dir>/<robot_id>/<whatever>.sdf
            # this would then be uploaded to the storage and referenced in bibi
            if not os.path.exists(os.path.join(simDir, robot_id)):
                os.mkdir(os.path.join(simDir, robot_id))
            else:
                logger.info("Copying robot in existing directory")

            from shutil import copy2
            copy2(sdf_abs_path, os.path.join(simDir, robot_id))

            # set the SDF path to be added to the RobotManager
            # notice this path isn't the extracted zip or template, but the <simDir>/<robot_id>
            robot_sdf_abs_path = os.path.join(simDir, robot_id, sdf_filename)

            # TODO: modify topic names here
            robot_sdf_abs_path = self._customize_sdf(robot_sdf_abs_path, robot_id)

            # add to the robot manager
            robot = Robot(robot_id, robot_sdf_abs_path, robot_id, pose, is_custom)
            self._cle_assembly.robotManager.add_robot(robot)
            self._cle_assembly.cle_server.cle.initial_robot_poses[robot_id] = pose

            # now try to add it to the scene
            try:
                self._cle_assembly.robotManager.load_robot_in_scene(robot_id)
            except Exception as e:
                # couldn't add into the scene, fallback
                # ideally you want delete the created folders
                self._cle_assembly.robotManager.remove_robot(robot_id)
                del self._cle_assembly.cle_server.cle.initial_robot_poses[robot_id]
                raise
        # pylint: disable=broad-except
        except Exception as e:
            return False, str(e)

        # Yey! copy the SDF into the experiment folder
        with open(robot_sdf_abs_path, 'r') as sdf:
            data = sdf.read()
        self._cle_assembly.storage_client.create_or_update(
            self._cle_assembly.token,
            self._cle_assembly.experiment_id,
            os.path.join(robot_id, sdf_filename),
            data,
            "application/octet-stream"
        )

        return True, os.path.join(robot_id, sdf_filename)

    def _customize_sdf(self, sdf_abs_path, robot_id):
        """

        :param sdf_abs_path: location of the file to be altered
        :param robot_id: id of the robot
        :return: path to the altered sdf
        """
        # pylint: disable=unused-argument,no-self-use
        # TODO: alter topic names in the robot sdf, in place or in a copy
        # return altered sdf location
        return sdf_abs_path
