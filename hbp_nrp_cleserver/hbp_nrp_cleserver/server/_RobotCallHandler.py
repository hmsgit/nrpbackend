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

import os
import logging
import tf
from pyxb import ValidationError

from cle_ros_msgs import msg
from hbp_nrp_cle.robotsim.RobotManager import Robot
from hbp_nrp_commons.sim_config.SimConfUtil import SimConfUtil
from hbp_nrp_commons.generated import robot_conf_api_gen as robotXmlParser
from hbp_nrp_commons.ZipUtil import ZipUtil
from hbp_nrp_commons.sim_config.SimConfig import ResourceType
from hbp_nrp_commons.workspace.Settings import Settings
from hbp_nrp_commons.workspace.SimUtil import SimUtil
from hbp_nrp_backend.storage_client_api.StorageClient import Model

__author__ = 'Hossain Mahmud'

logger = logging.getLogger(__name__)


class RobotCallHandler(object):
    """
    Helper class for ROSCLEServer to handle robot operations
    """

    def __init__(self, assembly):
        self._cle_assembly = assembly
        self._client = self._cle_assembly.storage_client
        self._simdir = self._cle_assembly.sim_dir

    def get_robots(self):
        """
        Gets robots list in the currently running simulation
        REST request GET /robots/<sim_id> ends up here

        :return: A list containing rospy msg object of the robot list
        """
        ret = []
        for rid, robot in self._cle_assembly.robotManager.get_robot_dict().iteritems():
            # convert quaternion pose to euler
            if not robot.pose:
                pose = None
            else:
                quaternion = (
                    robot.pose.orientation.x,
                    robot.pose.orientation.y,
                    robot.pose.orientation.z,
                    robot.pose.orientation.w
                )
                euler = tf.transformations.euler_from_quaternion(quaternion)
                pose = msg.Pose(x=robot.pose.position.x,
                                y=robot.pose.position.y,
                                z=robot.pose.position.z,
                                roll=euler[0],
                                pitch=euler[1],
                                yaw=euler[2]
                                )
            # SDFFileAbsPath contains a absolute path
            ret.append(msg.RobotInfo(robot_id=str(rid),
                                     robot_model=str(robot.SDFFileAbsPath),
                                     is_custom=bool(robot.isCustom),
                                     pose=pose)
                       )

        return ret

    def add_robot(self, robot_id, robot_model, is_custom=False, pose=None):
        """
        Adds a robot in the currently running simulation
        REST request POST /robots/<sim_id> ends up here

        If is_custom is set to True, the custom robot files must be available
        in robot_model_rel_path.

        :param robot_id: Id of the robot
        :param robot_model: if it is custom, the name of the models based the robot
                            if it is not custom SDF
        :param is_custom: is the asset custom (zip or sdf !)
        :param pose: initial robot pose
        :return: Tuple (True, SDF relative path) or (False, error message) to update config files
        """
        # pylint: disable=too-many-locals,too-many-branches,too-many-statements
        robot_sdf_abs_path = None
        try:
            if is_custom:  # pragma: no cover
                robot = Model(robot_model, ResourceType.ROBOT)
                zip_storage_path = self._client.get_model_path(
                    self._cle_assembly.sim_config.token,
                    self._cle_assembly.sim_config.ctx_id,
                    robot
                )
                try:
                    # It is assumed that prepare_custom_robot is called by this point
                    # Hence, files should be present already
                    zip_abs_path = os.path.join(self._simdir, zip_storage_path)

                    # get the root directory within the zip
                    root_folder = ZipUtil.get_rootname(zip_abs_path)
                    sdf_abs_path = self.get_sdf_abs_path(
                        os.path.join(self._cle_assembly.simAssetsDir, root_folder, 'model.config'))

                    if sdf_abs_path is None:
                        return False, "'model.config' has no 'sdf' tag"

                    sdf_filename = os.path.basename(sdf_abs_path)

                    if not os.path.isfile(sdf_abs_path):
                        return False, "No SDF named {name} found at {loc} specified " \
                                      "in model.config in the uploaded zip"\
                            .format(name=sdf_filename, loc=root_folder)
                # pylint: disable=broad-except
                except Exception as e:
                    return False, str(e)

            else:  # if template robot
                # find the SDF in the template directories
                sdf_abs_path = SimUtil.find_file_in_paths(
                    robot_model, [Settings.nrp_models_directory])
                if not sdf_abs_path:
                    raise Exception("Could not find {0} in the template library"
                                    .format(robot_model))
                sdf_filename = os.path.basename(sdf_abs_path)

            pose = SimConfUtil.convertXSDPosetoPyPose(pose)

            # copy sdf to <simulation dir>/<robot_id>/<whatever>.sdf
            # this would then be uploaded to the storage and referenced in bibi
            if not os.path.exists(os.path.join(self._simdir, robot_id)):
                os.mkdir(os.path.join(self._simdir, robot_id))
            else:
                logger.info("Copying robot in existing directory")

            from shutil import copy2
            copy2(sdf_abs_path, os.path.join(self._simdir, robot_id))

            # Copy available rosLaunch files from the directory of the sdf_abs_path
            # Take the first one (by name) if multiple available
            ros_launch_file = next((f for f in os.listdir(os.path.dirname(sdf_abs_path))
                                    if f.endswith('.launch')), None)
            ros_launch_abs_path = (ros_launch_file if ros_launch_file is None
                                   else os.path.join(os.path.dirname(sdf_abs_path), ros_launch_file)
                                   )

            # copy to simulation directory
            if ros_launch_abs_path is not None and os.path.isfile(ros_launch_abs_path):
                copy2(ros_launch_abs_path, os.path.join(self._simdir, robot_id))
                ros_launch_abs_path = os.path.join(self._simdir, robot_id,
                                                   os.path.basename(ros_launch_abs_path))

            # set the SDF path to be added to the RobotManager
            # notice this path isn't the extracted zip or template, but the <simDir>/<robot_id>
            robot_sdf_abs_path = os.path.join(self._simdir, robot_id, sdf_filename)

            # TODO: modify topic names here
            robot_sdf_abs_path = self._customize_sdf(robot_sdf_abs_path, robot_id)

            # add to the robot manager
            robot = Robot(robot_id, robot_sdf_abs_path, robot_id,
                          pose, is_custom, ros_launch_abs_path, robot_model)
            self._cle_assembly.cle_server.cle.initial_robot_poses[robot_id] = pose

            # now try to add it to the scene
            try:
                self._cle_assembly.robotManager.add_robot(robot)
            except Exception:
                # couldn't add, fallback! ideally you want delete the created folders
                # self._cle_assembly.robotManager.remove_robot(robot_id)
                del self._cle_assembly.cle_server.cle.initial_robot_poses[robot_id]
                raise
        # pylint: disable=broad-except
        except Exception as e:
            return False, str(e)

        # Yey! Upload the SDF into the storage experiment folder
        with open(robot_sdf_abs_path, 'r') as sdf:
            data = sdf.read()
        self._client.create_or_update(
            self._cle_assembly.sim_config.token,
            self._cle_assembly.sim_config.experiment_id,
            os.path.join(robot_id, sdf_filename),
            data,
            "application/octet-stream"
        )
        # Upload ros_launch file
        if ros_launch_abs_path is not None:
            with open(ros_launch_abs_path, 'r') as ros_launch:
                data = ros_launch.read()
            self._client.create_or_update(
                self._cle_assembly.sim_config.token,
                self._cle_assembly.sim_config.experiment_id,
                os.path.join(robot_id, os.path.basename(ros_launch_abs_path)),
                data,
                "application/octet-stream"
            )

        return True, os.path.join(robot_id, sdf_filename)

    def delete_robot(self, robot_id):
        """
        Delete a robot from the currently running simulation
        REST request DELETE /robots/<sim_id> ends up here

        :param robot_id: Id of the robot
        :return: Tuple (True, SDF relative path) or (False, error message) to update config files
        """
        try:
            modelFile = self._cle_assembly.robotManager.get_robot(robot_id).SDFFileAbsPath

            # delete model file from the storage
            self._client.delete_file(
                self._cle_assembly.sim_config.token,
                self._cle_assembly.sim_config.experiment_id,
                robot_id
            )
            # delete model from the simulation dir
            os.remove(modelFile)

        # pylint: disable=broad-except
        except Exception as e:
            # couldn't delete into the scene, fallback
            return False, "An error occurred while deleting the robot: {err}. {oldexp}" \
                .format(err=str(e),
                        oldexp="You may have an older copy of the template experiments."
                               "Please update your Experiments repository." if robot_id == 'robot'
                        else "")

        try:
            self._cle_assembly.robotManager.delete_robot_from_scene(robot_id)
            # finally delete from the manager
            self._cle_assembly.robotManager.remove_robot(robot_id)

        # pylint: disable=broad-except
        except Exception as e:
            # couldn't delete into the scene, fallback
            return False, "An error occurred while deleting the robot. {err}".format(err=str(e))

        return True, "Robot deleted"

    def prepare_custom_robot(self, robot_model):
        """
        Downloads and extracts a custom robot to the simulation directory
        :param name: name of the robot in the DB.
        :return: Tuple (True, extracted SDF absolute path) or (False, error message)
        """
        try:
            # download zip from storage
            zipAbsPath = self.download_custom_robot(robot_model)

            if zipAbsPath is None:
                raise Exception("Could not find {0} in the template library"
                                .format(robot_model))
            # extract assets
            ZipUtil.extractall(
                zip_abs_path=zipAbsPath,
                extract_to=self._cle_assembly.simAssetsDir,
                overwrite=True)
            # get the root directory within the zip
            rootfolder = ZipUtil.get_rootname(zipAbsPath)
            sdf_abs_path = self.get_sdf_abs_path(
                os.path.join(self._cle_assembly.simAssetsDir, rootfolder, 'model.config'))

            if sdf_abs_path is None:
                return False, "'model.config' has no 'sdf' tag"

            sdf_filename = os.path.basename(sdf_abs_path)

            if not os.path.isfile(sdf_abs_path):
                return False, "No SDF named {name} found at {loc} specified " \
                              "in model.config in the uploaded zip" \
                    .format(name=sdf_filename, loc=rootfolder)
        # pylint: disable=broad-except
        except Exception as e:
            logger.error("An error occurred while preparing custom model " + str(e))
            return False, str(e)

        return True, sdf_abs_path

    def download_custom_robot(self, robot_name, save_to=None, save_as=None):
        """
        Downloads custom zipped robot model from the storage
        :param name: name of the robot in the DB.
        * from the internal robot folder
        :param save_to: download location
        :param save_as: filename to save as
        :return: None if failed. Otherwise path to the downloaded file i.e. save_to+save_as
        """

        # download zip from storage
        try:

            robot = Model(robot_name, ResourceType.ROBOT)
            data = self._client.get_model(
                self._cle_assembly.sim_config.token,
                self._cle_assembly.sim_config.ctx_id,
                robot
            )

            if save_as is None:
                path = self._client.get_model_path(
                    self._cle_assembly.sim_config.token,
                    self._cle_assembly.sim_config.ctx_id,
                    robot)
                save_as = os.path.basename(path)
                save_to = os.path.join(self._simdir, os.path.dirname(path))
        # pylint: disable=broad-except
        except Exception as ex:
            logger.error("StorageClient failed to download {zip} with following error. {err}"
                         .format(zip=robot_name, err=str(ex)))
            return None

        # write the zip
        try:
            if not os.path.exists(save_to):
                os.makedirs(save_to)
            with open(os.path.join(save_to, save_as), 'w') as destAbsPath:
                destAbsPath.write(data)
        except IOError as ex:
            logger.error("Could not write downloaded SDF {sdf} in directory {dir} due to {err}"
                         .format(sdf=data.path, dir=save_to, err=str(ex)))
            return None

        return os.path.join(save_to, save_as)

    def get_sdf_abs_path(self, model_config_abs_path):
        """
        Get robot sdf name from model.config
        :param model_config_abs_path:
        :return:
        """
        # pylint: disable=no-self-use
        # Get robot sdf name from model.config
        with open(model_config_abs_path) as conf:
            try:
                confDOM = robotXmlParser.CreateFromDocument(conf.read())
            except ValidationError as ve:
                raise Exception("Could not parse experiment configuration {0:s} due "
                                "to validation error: {1:s}".format(conf, str(ve)))

        if confDOM.sdf is None:
            return None

        return os.path.join(os.path.dirname(model_config_abs_path), confDOM.sdf.value())

    def _customize_sdf(self, sdf_abs_path, robot_id):  # pragma: no cover
        """

        :param sdf_abs_path: location of the file to be altered
        :param robot_id: id of the robot
        :return: path to the altered sdf
        """
        # pylint: disable=unused-argument,no-self-use
        # TODO: alter topic names in the robot sdf, in place or in a copy
        # return altered sdf location
        return sdf_abs_path
