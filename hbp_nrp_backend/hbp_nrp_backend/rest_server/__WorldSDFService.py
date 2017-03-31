"""
This module contains the REST services to download sdf files
describing the world in which a simulation is carried out.
"""

__author__ = 'UgoAlbanese'

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, \
     NRPServicesUnavailableROSService
from gazebo_msgs.srv import ExportWorldSDF
from flask_restful import Resource
from flask_restful_swagger import swagger
import logging
import rospy
from lxml import etree as ET

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
            raise NRPServicesClientErrorException(
                "Service did not process request:" + str(exc))

        return {"sdf": sdf_string}, 200
