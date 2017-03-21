"""
This module contains the REST implementation
for loading (and someday writing) experiment preview images
"""


__author__ = 'Bernd Eckstein'

from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.RestSyncMiddleware import RestSyncMiddleware
from hbp_nrp_backend.rest_server.__ExperimentService import get_experiment_rel, \
    get_experiment_basepath, ErrorMessages

from hbp_nrp_commons.generated import exp_conf_api_gen
from pyxb import ValidationError

import mimetypes
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
                "message": ErrorMessages.MODEXP_VARIABLE_ERROR
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
    @RestSyncMiddleware.threadsafe
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
        experiment_file_path = os.path.join(get_experiment_basepath(),
                                            get_experiment_rel(exp_id))
        if not os.path.isfile(experiment_file_path):
            raise NRPServicesClientErrorException(ErrorMessages.EXPERIMENT_CONF_FILE_NOT_FOUND_404,
                                                  error_code=404)

        experiment_dir = os.path.split(experiment_file_path)[0]
        # Parse the experiment XML and get the thumbnail path
        with open(experiment_file_path) as exd_file:
            try:
                experiment_file = exp_conf_api_gen.CreateFromDocument(exd_file.read())
                preview_file = os.path.join(experiment_dir, experiment_file.thumbnail)
            except ValidationError:
                raise NRPServicesClientErrorException(
                                                    ErrorMessages.EXPERIMENT_CONF_FILE_INVALID_500,
                                                    error_code=500)

        # Check thumbnail
        if not os.path.isfile(preview_file):
            raise NRPServicesClientErrorException(ErrorMessages.EXPERIMENT_PREVIEW_NOT_FOUND_404,
                                                  error_code=404)
        mime_type = mimetypes.guess_type(preview_file)[0] # returns tuple (type, encoding)
        if not mime_type or not mime_type.startswith('image'):
            raise NRPServicesClientErrorException(ErrorMessages.EXPERIMENT_PREVIEW_INVALID_500,
                                                  error_code=500)

        with open(preview_file, "rb") as _file:
            data = _file.read()

        # Base64
        return dict(image_as_base64=base64.b64encode(data))
