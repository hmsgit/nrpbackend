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
This module provides support methods to manipulate exc and bibi files.
"""
import logging
from hbp_nrp_commons.generated import bibi_api_gen as bibi_parser, exp_conf_api_gen as exc_parser

__author__ = 'Hossain Mahmud'

logger = logging.getLogger(__name__)


class ConfigEditor(object):  # pragma: no cover
    """
    'Friend' class for SimConfig to manipulate (in-memory and storage) exc and bibi
    """

    def __init__(self, sim_config):
        self._sim_config = sim_config

        # Note: the rationale of separating this class from the SimConfig is to keep SimConfig
        # strictly as a model. But since we are restricting direct access to the dom objects
        # we had make exceptions here

        # pylint: disable=protected-access
        self._exc_dom = sim_config._exc_dom
        self._bibi_dom = sim_config._bibi_dom

    def add_robotpose(self, robot_id, pose=None):
        """
        Adds a <robotPose> tag in the exc

        :param robot_id: robotId attribute for the tag
        :param pose: A cle_ros_msgs.msgs.Pose object (defines an object's Euler pos and orientation)
        :return:
        """
        # RobotPose is the type of <robotPose> defined in the exc schema (xsd) file in Experiments
        tag = exc_parser.RobotPose()
        tag.robotId = robot_id
        if pose is None:
            tag.x = 0.0
            tag.y = 0.0
            tag.z = 0.0
            tag.roll = 0.0
            tag.pitch = 0.0
            tag.yaw = 0.0
        else:
            tag.x = pose.x
            tag.y = pose.y
            tag.z = pose.z
            tag.roll = pose.roll
            tag.pitch = pose.pitch
            tag.yaw = pose.yaw

        self._exc_dom.environmentModel.append(tag)

        # HACK: Replace exc and bibi representation by reading from storage
        rp = self._exc_dom.environmentModel.robotPose
        self._hack_()
        self._exc_dom.environmentModel.robotPose = rp
        # END HACK

        # Update sim dir copy of the exc
        self._write_xml(self._exc_dom.toxml('utf-8'), self._sim_config.exc_abs_path)

    def delete_robotpose(self, robot_id):
        """
        Deletes <robotPose> tag in the exc where robotId is robot_id

        :param robot_id: robotId attribute for the tag
        :return:
        """

        del_index = None
        for i in range(len(self._exc_dom.environmentModel.robotPose)):
            if self._exc_dom.environmentModel.robotPose[i].robotId == robot_id:
                del_index = i
                break

        if del_index is not None:
            del self._exc_dom.environmentModel.robotPose[del_index]

        # HACK: Replace exc and bibi representation by reading from storage
        rp = self._exc_dom.environmentModel.robotPose
        self._hack_()
        self._exc_dom.environmentModel.robotPose = rp
        # END HACK

        # Update sim dir copy of the exc
        self._write_xml(self._exc_dom.toxml('utf-8'), self._sim_config.exc_abs_path)

    def update_robotpose(self, robot_id, pose):
        """
        Edit <robotPose> tag in the exc where robotId is robot_id

        :param robot_id: robotId attribute for the tag
        :param pose: A cle_ros_msgs.msgs.Pose object (defines an object's Euler pos and orientation)
        :return: Tuple (True, SDF relative path) or (False, error message) to update config files
        """

        if not robot_id in self._sim_config.robot_models:
            return False, "No robot exists with id equals {id}".format(id=robot_id)

        for tag in self._exc_dom.environmentModel.robotPose:
            if tag.robotId == robot_id:
                tag.x = pose.x
                tag.y = pose.y
                tag.z = pose.z
                tag.roll = pose.roll
                tag.pitch = pose.pitch
                tag.yaw = pose.yaw

        # HACK: Replace exc and bibi representation by reading from storage
        rp = self._exc_dom.environmentModel.robotPose
        self._hack_()
        self._exc_dom.environmentModel.robotPose = rp
        # END HACK

        # Update sim dir copy of the exc
        self._write_xml(self._exc_dom.toxml('utf-8'), self._sim_config.exc_abs_path)

        return True, "Tag updated successfully"

    def add_bodymodel(self, robot_id, model_path, is_custom, robot_model=None):
        """
        Adds a <bodyModel> tag in the bibi

        :param robot_id: attribute robotId in the tag
        :param model_path: value() fo the tag
        :param is_custom: attribute customAsset in the tag
        :param zip_path: if custom, then value of the assetPath attribute
        :param robot_model_name: if custom, the name of the custom model is based the robot
        :return:
        """
        # SDFWithPath is the type of <bodyModel> defined in the bibi schema (xsd) file
        tag = bibi_parser.SDFWithPath(model_path)
        tag.robotId = robot_id
        tag.isCustom = "true" if is_custom else "false"

        if is_custom and robot_model is not None:
            tag.model = robot_model

        if self._bibi_dom.bodyModel:
            self._bibi_dom.bodyModel.append(tag)
        else:
            self._bibi_dom.append(tag)

        # HACK: Replace exc and bibi representation by reading from storage
        bm = self._bibi_dom.bodyModel
        self._hack_()
        self._bibi_dom.bodyModel = bm
        # END HACK

        # Update sim dir copy of the bibi
        self._write_xml(self._bibi_dom.toxml('utf-8'), self._sim_config.bibi_path.abs_path)

    def delete_bodymodel(self, robot_id):
        """
        Deletes a <bodyModel> tag from the bibi

        :param robot_id: attribute robotId in the tag
        :return:
        """

        del_index = None
        for i in range(len(self._bibi_dom.bodyModel)):
            if self._bibi_dom.bodyModel[i].robotId == robot_id:
                del_index = i
                break

        if del_index is not None:
            del self._bibi_dom.bodyModel[del_index]

        # HACK: Replace exc and bibi representation by reading from storage
        bm = self._bibi_dom.bodyModel
        self._hack_()
        self._bibi_dom.bodyModel = bm
        # END HACK

        # Update sim dir copy of the exc
        self._write_xml(self._bibi_dom.toxml('utf-8'), self._sim_config.bibi_path.abs_path)

    def _prettify_xml(self, plain_text):
        """
        Format a given xml text

        :param plain_text: xml text string
        :return: formatted xml string
        """

        # pylint: disable=no-self-use
        import lxml
        return lxml.etree.tostring(lxml.etree.XML(plain_text), pretty_print=True)

    def _write_xml(self, plain_text, filename):
        """
        Write xml into file. If the file exists, it overwrites the content.

        :param plain_text: xml text
        :param filename: absolute path to the file to write
        :return: Tuple (True, None) or (False, error)
        """

        # pylint: disable=no-self-use, broad-except
        try:
            with open(filename, 'w') as f:
                try:
                    f.write(self._prettify_xml(plain_text))
                except IOError as e:
                    return False, str(e)
        except Exception as e:
            return False, str(e)

        return True, None

    def _hack_(self):
        """
        TODO: please remove me at earliest convenience

        HACK for the inconsistent proxy and backend behavior
        Replace exc and bibi representation by reading from storage
        Proxy might have changed something that backend is not aware of
        """
        from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient
        import os
        import urllib

        client = StorageClient()

        latest_exc = client.get_file(
            self._sim_config.token, urllib.quote_plus(self._sim_config.experiment_id),
            os.path.basename(self._sim_config.exc_abs_path), by_name=True
        )
        latest_bibi = client.get_file(
            self._sim_config.token, urllib.quote_plus(self._sim_config.experiment_id),
            os.path.basename(self._sim_config.bibi_path.rel_path), by_name=True
        )

        try:
            self._exc_dom = exc_parser.CreateFromDocument(latest_exc)
            self._bibi_dom = bibi_parser.CreateFromDocument(latest_bibi)

        except Exception as ex:
            raise Exception("Something went horribly wrong while creating latest "
                            "config objects with following exception {}".format(str(ex)))
