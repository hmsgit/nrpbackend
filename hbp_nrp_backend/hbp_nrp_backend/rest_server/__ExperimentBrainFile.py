"""
This module contains the REST implementation
for loading and saving experiment brain files
"""


__author__ = 'Bernd Eckstein'

from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import \
    ErrorMessages, get_brain_file

import os
import base64

# pylint: disable=R0201
# because it seems to be buggy:
# pylint: disable=W0105


class ExperimentBrainFile(Resource):
    """
    The resource to load experiment brain files
    """

    @swagger.model
    class _brain(object):
        """
        Get and Set Experiment brain
        Only used for swagger documentation
        """

        resource_fields = {
            'brain_type': fields.String(),
            'filename': fields.String(),
            'data_type': fields.String(),
            'data': fields.String()
        }
        required = ['brain_type', 'filename', 'data_type', 'data']

    @swagger.operation(
        notes='Get the brain file of a given experiment.',
        responseClass=_brain.__name__,
        parameters=[
            {
                "name": "exp_id",
                "description": "The ID of the experiment brain file to be retrieved",
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
                "code": 500,
                "message": ErrorMessages.EXPERIMENT_BRAIN_FILE_NOT_FOUND_500
            },
            {
                "code": 404,
                "message": ErrorMessages.EXPERIMENT_NOT_FOUND_404
            },
            {
                "code": 200,
                "message": "Success. The experiment brain file was retrieved"
            }
        ]
    )
    def get(self, exp_id):
        """
        Gets brain file of the experiment specified with experiment ID.
        Depending of the type of brain file, it is transmitted as text or
        as base64 (given in the field data_type)

        :param exp_id: The experiment ID
        :>json string brain_type: Type of the brain file ("h5" or "py")
        :>json string filename: Name of the experiment file
        :>json string data_type: type of the data field ('text' or 'base64')
        :>json string data: Contents of the brain file. Encoding given in field data_type
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 500: The experiment brain file was not found
        :status 404: The experiment with the given ID was not found
        :status 200: Success. The experiment brain file was retrieved
        """

        filename = get_brain_file(exp_id)

        if not os.path.isfile(filename):
            raise NRPServicesClientErrorException(
                ErrorMessages.EXPERIMENT_BRAIN_FILE_NOT_FOUND_500, error_code=500)

        # Get file ending ('py' or 'h5')
        brain_type = os.path.splitext(filename)[1][1:]

        with open(filename, "rb") as _file:
            if brain_type == "py":
                data = _file.read()
                data_type = "text"
            elif brain_type == "h5":
                data = base64.b64encode(_file.read())
                data_type = "base64"
            else:
                data = None
                data_type = None

        return {'filename': filename,
                'brain_type': brain_type,
                'data_type': data_type,
                'data': data}, 200
