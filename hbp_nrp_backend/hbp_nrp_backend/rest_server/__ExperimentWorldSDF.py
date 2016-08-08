"""
This module contains the REST implementation
for saving experiment environment files
"""

__author__ = 'Luc Guyot, Daniel Peppicelli'

import logging
import os
import rospy
import shutil
import tempfile
from threading import Thread
import tf.transformations

from lxml import etree as ET
from flask import request
from flask_restful import Resource
from flask_restful_swagger import swagger
from gazebo_msgs.srv import ExportWorldSDF
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, \
    NRPServicesUnavailableROSService, NRPServicesGeneralException
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages
from hbp_nrp_commons.generated import exp_conf_api_gen

logger = logging.getLogger(__name__)


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
            raise NRPServicesClientErrorException(
                "Service did not process request:" + str(exc))

        client = NeuroroboticsCollabClient(UserAuthentication.get_header_token(request),
                                           context_id)
        replace_sdf = Thread(target=client.replace_file_content_in_collab,
                             args=(sdf_string,
                                   NeuroroboticsCollabClient.SDF_WORLD_MIMETYPE,
                                   NeuroroboticsCollabClient.SDF_WORLD_FILE_NAME,
                                   "recovered_world.sdf"))
        replace_sdf.start()

        # Save the robot position in the ExDConf file
        if (len(robot_pose) is 6):  # We need 6 elements (from Gazebo)
            experiment_configuration_file_path = client.clone_file_from_collab_context(
                client.EXPERIMENT_CONFIGURATION_MIMETYPE,
                client.EXPERIMENT_CONFIGURATION_FILE_NAME)
            if experiment_configuration_file_path is None:
                raise NRPServicesGeneralException(
                    "Experiment configuration file not found in the Collab storage",
                    "Experiment configuration not found"
                )

            experiment_configuration = None
            with open(experiment_configuration_file_path) as experiment_configuration_xml:
                experiment_configuration = exp_conf_api_gen.CreateFromDocument(
                    experiment_configuration_xml.read())
                if not isinstance(experiment_configuration, exp_conf_api_gen.ExD_):
                    raise NRPServicesGeneralException(
                        "Experiment configuration "
                        "file content is not valid.",
                        "Experiment configuration not valid"
                    )

            experiment_configuration.environmentModel.robotPose.x = robot_pose[0]
            experiment_configuration.environmentModel.robotPose.y = robot_pose[1]
            experiment_configuration.environmentModel.robotPose.z = robot_pose[2]
            quaternion = tf.transformations.quaternion_from_euler(float(robot_pose[3]),
                                                                  float(robot_pose[4]),
                                                                  float(robot_pose[5]))
            experiment_configuration.environmentModel.robotPose.ux = quaternion[0]
            experiment_configuration.environmentModel.robotPose.uy = quaternion[1]
            experiment_configuration.environmentModel.robotPose.uz = quaternion[2]
            experiment_configuration.environmentModel.robotPose.theta = quaternion[3]

            if tempfile.gettempdir() in experiment_configuration_file_path:
                logger.debug(
                    "removing the temporary experiment configuration file %s",
                    experiment_configuration_file_path
                )
                shutil.rmtree(os.path.dirname(experiment_configuration_file_path))
            client.replace_file_content_in_collab(
                experiment_configuration.toxml("utf-8"),
                client.EXPERIMENT_CONFIGURATION_MIMETYPE,
                client.EXPERIMENT_CONFIGURATION_FILE_NAME,
                "recovered_experiment_configuration.xml"
            )

        else:
            logger.error("Malformed robot position tag in SDF: " + robot_pose)

        replace_sdf.join()
        return 200
