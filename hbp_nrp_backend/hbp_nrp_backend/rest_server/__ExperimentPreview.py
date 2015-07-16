"""
This module contains the REST implementation
for loading (and someday writing) experiment preview images
"""


__author__ = 'Bernd Eckstein'

from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import get_experiment_rel, get_basepath, \
    ErrorMessages

import base64
import os

# pylint: disable=R0201


class ExperimentPreview(Resource):
    """
    Get Experiment preview PNG file as octet-stream
    """

    @swagger.model
    class _Image(object):
        """
        Get and Set Experiment Configuration
        Only used for swagger documentation
        """

        resource_fields = {
            'image_as_base64': fields.String()
        }
        required = ['image_as_base64']

    @swagger.operation(
        notes='Get the preview image of a given experiment',
        responseClass=_Image.__name__,
        parameters=[
            {
                "name": "exp_id",
                "description": "The ID of the experiment",
                "required": True,
                "paramType": "path",
                "dataType": basestring.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.VARIABLE_ERROR
            },
            {
                "code": 404,
                "message": ErrorMessages.EXPERIMENT_NOT_FOUND_404
            },
            {
                "code": 404,
                "message": ErrorMessages.EXPERIMENT_PREVIEW_NOT_FOUND_404
            },
            {
                "code": 200,
                "message": "Success. The preview image was sent."
            }
        ]
    )
    def get(self, exp_id):
        """
        Get preview image of the experiment specified with experiment ID.

        :param exp_id: The experiment ID
        :>json image_as_base64: The PNG image as base64
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The experiment with the given ID was not found
        :status 404: The experiment has no preview image
        :status 200: Success. The preview image is returned
        """

        # Check experiment
        experiment_file = get_experiment_rel(exp_id)

        preview_file = os.path.join(get_basepath(), os.path.splitext(
            experiment_file)[0] + ".png")

        if not os.path.isfile(preview_file):
            raise NRPServicesClientErrorException(ErrorMessages.EXPERIMENT_PREVIEW_NOT_FOUND_404,
                                                  404)

        with open(preview_file, "rb") as _file:
            data = _file.read()

        # Base64
        return dict(image_as_base64=base64.b64encode(data))
