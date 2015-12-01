"""
This module contains the REST implementation
for saving experiment environment files
"""

__author__ = 'Luc Guyot, Daniel Peppicelli'

import rospy
from lxml import etree as ET
from flask import request
from flask_restful import Resource
from flask_restful_swagger import swagger
from gazebo_msgs.srv import ExportWorldSDF
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, \
    NRPServicesUnavailableROSService
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages


# pylint: disable=no-self-use
class ExperimentWorldSDF(Resource):
    """
    The resource representing "saved" environment files (SDF).
    """

    @swagger.operation(
        notes='Save the current running experiment SDF back to the collab storage.',
        parameters=[
            {
                "name": "context_id",
                "description": "The UUID of the Collab context to be paired with \
                the ID of the selected experiment",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.ERROR_SAVING_FILE_500
            },
            {
                "code": 200
            }
        ]
    )
    def put(self, context_id):
        """
        Save the current running experiment SDF back to the collab storage

        :param context_id: The collab context ID
        :status 500: Error saving file
        :status 200: Success. File written.
        """

        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient

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

        client = NeuroroboticsCollabClient(UserAuthentication.get_header_token(request),
                                           context_id)
        client.save_string_to_file_in_collab(sdf_string,
                                             NeuroroboticsCollabClient.SDF_WORLD_MIMETYPE,
                                             "recovered_world.sdf")

        return 200
