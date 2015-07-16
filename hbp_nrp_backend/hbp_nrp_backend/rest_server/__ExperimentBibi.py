"""
This module contains the REST implementation
for loading and saving experiment bibi files
"""


__author__ = 'Bernd Eckstein'

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import save_file, \
    ErrorMessages, get_bibi_file

import os
import base64

# pylint: disable=R0201
# because it seems to be buggy:
# pylint: disable=W0105


class ExperimentBibi(Resource):
    """
    The resource to load and save experiment bibi files
    """

    @swagger.model
    class _Bibi(object):
        """
        Get and Set Experiment BIBI
        Only used for swagger documentation
        """

        resource_fields = {
            'filename': fields.String(),
            'base64': fields.String()
        }
        required = ['base64']

    @swagger.operation(
        notes='Get the bibi file of a given experiment.',
        responseClass=_Bibi.__name__,
        parameters=[
            {
                "name": "exp_id",
                "description": "The ID of the experiment BIBI file to be retrieved",
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
                "message": ErrorMessages.EXPERIMENT_FILE_NOT_FOUND_404
            },
            {
                "code": 200,
                "message": "Success. The experiment BIBI file was retrieved"
            }
        ]
    )
    def get(self, exp_id):
        """
        Gets bibi file of the experiment specified with experiment ID.

        :param exp_id: The experiment ID
        :>json string filename: Name of the experiment file
        :>json string base64: Contents of the BIBI file encoded as base64
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The experiment with the given ID was not found
        :status 404: The experiment BIBI file was not found
        :status 200: Success. The experiment BIBI file was retrieved
        """

        filename = get_bibi_file(exp_id)

        if not os.path.isfile(filename):
            raise NRPServicesClientErrorException(ErrorMessages.EXPERIMENT_FILE_NOT_FOUND_404, 404)

        with open(filename, "rb") as _file:
            data = base64.b64encode(_file.read())

        return {'filename': filename, 'base64': data}, 200

    @swagger.operation(
        notes='Sends a BIBI file of an experiment to the server',
        parameters=[
            {
                "name": "exp_id",
                "required": True,
                "description": "The ID of the experiment, whose BIBI file will be written.",
                "paramType": "path",
                "dataType": basestring.__name__
            },
            {
                "name": "base64",
                "description": "The base64 encoded string with the content of the file.",
                "required": True,
                "paramType": "body",
                "dataType": _Bibi.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.VARIABLE_ERROR
            },
            {
                "code": 500,
                "message": ErrorMessages.ERROR_SAVING_FILE_500
            },
            {
                "code": 404,
                "message": ErrorMessages.EXPERIMENT_NOT_FOUND_404
            },
            {
                "code": 400,
                "message": ErrorMessages.ERROR_IN_BASE64_400
            },
            {
                "code": 200,
                "message": "Success. File written: '[file_name]'"
            }
        ]
    )
    def put(self, exp_id):
        """
        Send a BIBI file of an experiment to the server
        (currently, the file will be written with the ending "_new")

        :param path exp_id: The experiment ID
        :<json body json string filename: Name of the BIBI file (currently not used)
        :<json body json string base64: Contents of the file encoded as base64
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 500: Error saving file
        :status 404: The experiment with the given ID was not found
        :status 400: The given base64 has an error
        :status 200: Success. File written.
        """

        body = request.get_json(force=True)
        encoded_data = body['base64']

        filename = get_bibi_file(exp_id)

        save_file(encoded_data, filename)
        return {'message': "Success. File written: '{0}'".format(filename)}, 200
