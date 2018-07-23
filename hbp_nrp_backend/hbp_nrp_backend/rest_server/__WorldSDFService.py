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
This module contains the REST services to download sdf files
describing the world in which a simulation is carried out.
"""

__author__ = 'UgoAlbanese'

import logging
import rospy
from lxml import etree as ET
import os
import xml

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, \
     NRPServicesUnavailableROSService
from gazebo_msgs.srv import ExportWorldSDF
from flask_restful import fields, Resource
from flask_restful_swagger import swagger
from flask import request
from hbp_nrp_backend.rest_server import ErrorMessages
from hbp_nrp_backend.__UserAuthentication import UserAuthentication
from hbp_nrp_commons.bibi_functions import docstring_parameter
from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages
from hbp_nrp_commons.generated import exp_conf_api_gen
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort

# pylint: disable=no-self-use

logger = logging.getLogger(__name__)


class WorldSDFService(Resource):
    """
    The sdf world file download service
    """

    @swagger.model
    class sdfData(object):
        """
        Represents environment sdf file
        Only used for swagger documentation
        """
        resource_fields = {'sdf': fields.String()}

        required = ['sdf']

    @swagger.operation(
        notes='Returns the SDF file describing the world without robots',
        responseClass=sdfData.__name__,
        responseMessages=[{
            "code": 500,
            "message": "ROS service not available"
        }, {
            "code": 400,
            "message": "A ROS error occurred"
        }, {
            "code": 200,
            "message": "SDF successfully returned"
        }])
    # pylint: disable=unused-argument
    def get(self, sim_id):
        """
        Returns the SDF file describing the world in which the simulation is carried out

        :> json string sdf: the SDF string describing the world excluding the robots involved

        :status 500: ROS service not available
        :status 400: A ROS error occurred
        :status 200: SDF file successfully returned
        """

        try:
            rospy.wait_for_service('/gazebo/export_world_sdf', 3)
        except rospy.ROSException as exc:
            raise NRPServicesUnavailableROSService(str(exc))

        dump_sdf_world = rospy.ServiceProxy('/gazebo/export_world_sdf', ExportWorldSDF)

        try:
            sdf_string = dump_sdf_world().sdf_dump
            tree = ET.fromstring(sdf_string)
            for m in tree.findall(".//model[@name='robot']"):
                m.getparent().remove(m)
            sdf_string = ET.tostring(tree, encoding='utf8', method='xml')
        except rospy.ServiceException as exc:
            raise NRPServicesClientErrorException("Service did not process request:" + str(exc))

        return {"sdf": sdf_string}, 200

    @swagger.operation(
        notes='Save the current running experiment SDF to the storage.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The Id of the simulation",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
        ],
        responseMessages=[{
            "code": 500,
            "message": ErrorMessages.ERROR_SAVING_FILE_500
        }, {
            "code": 200,
            "message": "Success. File written."
        }])
    @docstring_parameter(ErrorMessages.ERROR_SAVING_FILE_500)
    def post(self, sim_id):
        """
        Save the current running experiment SDF back to the storage
        :param sim_id: The sim_id
        :param context_id: The context_id of the experiment
        :status 500: Error saving file
        :status 200: Success. File written.
        """
        # pylint: disable=too-many-locals
        context_id = UserAuthentication.get_header_token(request)
        simulation = _get_simulation_or_abort(sim_id)

        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.storage_client_api.StorageClient \
            import StorageClient
        try:
            rospy.wait_for_service('/gazebo/export_world_sdf', 3)
        except rospy.ROSException as exc:
            raise NRPServicesUnavailableROSService(str(exc))

        dump_sdf_world = rospy.ServiceProxy('/gazebo/export_world_sdf', ExportWorldSDF)
        robot_pose = []

        try:
            sdf_string = dump_sdf_world().sdf_dump
            tree = ET.fromstring(sdf_string)
            try:
                robot_pose = tree.findall(".//state/model[@name='robot']/pose")[0].text.split()
            # pylint: disable=bare-except
            except:
                logger.error("Can't retrieve robot position.")
            # Erase all robots from the SDF
            for m in tree.findall(".//model[@name='robot']"):
                m.getparent().remove(m)
            sdf_string = ET.tostring(tree, encoding='utf8', method='xml')
        except rospy.ServiceException as exc:
            raise NRPServicesClientErrorException("Service did not process request:" + str(exc))

        client = StorageClient()

        # find the sdf world filename from the .exc
        exp_xml_file_path = client.clone_file('experiment_configuration.exc',
                                              UserAuthentication.get_header_token(request),
                                              simulation.experiment_id)

        experiment_file = client.parse_and_check_file_is_valid(exp_xml_file_path,
                                                               exp_conf_api_gen.CreateFromDocument,
                                                               exp_conf_api_gen.ExD_)

        world_file_name = experiment_file.environmentModel.src

        if 'storage://' in world_file_name:
            world_file_name = os.path.basename(world_file_name)
            client.create_or_update(
                UserAuthentication.get_header_token(request),
                client.get_folder_uuid_by_name(
                    UserAuthentication.get_header_token(request), context_id, 'environments'),
                    world_file_name,
                sdf_string, "text/plain")
        else:
            client.create_or_update(
                UserAuthentication.get_header_token(request), simulation.experiment_id,
                world_file_name, sdf_string, "text/plain")

        # Save the robot position in the ExDConf file
        if len(robot_pose) is 6:  # We need 6 elements (from Gazebo)
            experiment_file.environmentModel.robotPose.x = robot_pose[0]
            experiment_file.environmentModel.robotPose.y = robot_pose[1]
            experiment_file.environmentModel.robotPose.z = robot_pose[2]
            experiment_file.environmentModel.robotPose.roll = robot_pose[3]
            experiment_file.environmentModel.robotPose.pitch = robot_pose[4]
            experiment_file.environmentModel.robotPose.yaw = robot_pose[5]

            client.create_or_update(
                UserAuthentication.get_header_token(request), simulation.experiment_id,
                'experiment_configuration.exc', xml.dom.minidom.parseString(
                    experiment_file.toxml("utf-8")).toprettyxml(), "text/plain")

        else:
            logger.error("Malformed robot position tag in SDF: " + robot_pose)

        return 200
