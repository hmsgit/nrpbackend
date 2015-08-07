"""
This module contains the REST services to download sdf files
describing the world in which a simulation is carried out.
"""

__author__ = 'UgoAlbanese'

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from gazebo_msgs.srv import ExportWorldSDF
from flask import request
from flask_restful import Resource
from flask_restful_swagger import swagger
import logging
import rospy
from lxml import etree as ET
import tempfile

# pylint: disable=no-self-use

logger = logging.getLogger(__name__)


class WorldSDFService(Resource):
    """
    The sdf world file download service
    """

    @swagger.operation(
        notes='Returns the SDF file describing the world without robots',
        responseMessages=[
            {
                "code": 200,
                "message": "SDF successfully returned"
            },
            {
                "code": 400,
                "message": "A ROS error occurred"
            }
        ]
    )
    def get(self):
        """
        Returns the SDF file describing the world in which the simulation is carried out

        :>json string sdf: the SDF string describing the world excluding the robots involved
        :status 200: SDF file successfully returned
        :status 400: A ROS error occurred
        """

        try:
            rospy.wait_for_service('/gazebo/export_world_sdf', 1)
        except rospy.ROSException as exc:
            raise NRPServicesClientErrorException("ROS service not available: " + str(exc), 400)

        dump_sdf_world = rospy.ServiceProxy('/gazebo/export_world_sdf', ExportWorldSDF)

        try:
            sdf_string = dump_sdf_world().sdf_dump
            tree = ET.fromstring(sdf_string)
            for m in tree.findall(".//model[@name='robot']"):
                m.getparent().remove(m)
            sdf_string = ET.tostring(tree, encoding='utf8', method='xml')
        except rospy.ServiceException as exc:
            raise NRPServicesClientErrorException(
                "Service did not process request:" + str(exc), 400)

        return {"sdf": sdf_string}, 200

    @swagger.operation(
        notes='Saves the SDF file with the custom environment in a temporary folder.',
        responseMessages=[
            {
                "code": 200,
                "message": "SDF environment successfully saved"
            },
            {
                "code": 400,
                "message": "Invalid XML string"
            },
            {
                "code": 401,
                "message": "Couldn't create the temporary file"
            }
        ]
    )
    # pylint: disable=broad-except
    def put(self):
        """
        Saves the SDF file with the custom environment in the Model folder.

        :<json string sdf: the sdf describing the world
        :>json string path: the path where the file was saved
        :status 200: SDF environment successfully saved
        :status 400: Invalid XML string
        :status 500: Couldn't create the temporary file
        """

        body = request.get_json(force=True)
        sdf_string = body['sdf']

        try:
            ET.fromstring(sdf_string)
        except ET.XMLSyntaxError as e:
            raise NRPServicesClientErrorException(
                "Invalid XML format in 'sdf' argument: " + str(e), 400)

        try:
            with tempfile.NamedTemporaryFile(prefix='customenv_', dir='/tmp', delete=False) as f:
                f.write(sdf_string)
                return {'path': f.name}, 200
        except Exception as e:
            raise NRPServicesClientErrorException(
                "Couldn't create the temporary file: " + str(e), 500)
