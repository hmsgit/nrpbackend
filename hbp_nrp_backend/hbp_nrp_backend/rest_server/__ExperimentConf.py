"""
This module contains the REST implementation
for loading and saving experiment configuration files
"""


__author__ = 'Bernd Eckstein'

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import save_file, \
    ErrorMessages, get_experiment_conf

import os
import base64

# pylint: disable=R0201
# because it seems to be buggy:
# pylint: disable=W0105


class ExperimentConf(Resource):
    """
    The resource to load and save experiment configuration files
    """

    @swagger.model
    class _Conf(object):
        """
        Get and Set Experiment Configuration
        Only used for swagger documentation
        """

        resource_fields = {
            'filename': fields.String(),
            'base64': fields.String()
        }
        required = ['base64']

    @swagger.operation(
        notes='Get the configuration file of a given experiment.',
        responseClass=_Conf.__name__,
        parameters=[
            {
                "name": "exp_id",
                "description": "The ID of the experiment to be retrieved",
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
                "message": ErrorMessages.EXPERIMENT_CONF_FILE_NOT_FOUND_404
            },
            {
                "code": 200,
                "message": "Success. The experiment file was retrieved."
            }
        ]
    )
    def get(self, exp_id):
        """
        Gets configuration file of the experiment specified with experiment ID.

        :param exp_id: The experiment ID
        :>json string filename: Name of the experiment file
        :>json string base64: Contents of the file encoded as base64
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The experiment with the given ID was not found
        :status 404: The experiment configuration file was not found
        :status 200: Success. The data of the file is returned
        """

        filename = get_experiment_conf(exp_id)

        if not os.path.isfile(filename):
            raise NRPServicesClientErrorException(ErrorMessages.EXPERIMENT_CONF_FILE_NOT_FOUND_404,
                                                  error_code=404)

        with open(filename, "rb") as _file:
            data = base64.b64encode(_file.read())

        return {'filename': filename, 'base64': data}, 200

    @swagger.operation(
        notes='Sends a configuration file of an experiment to the server',
        parameters=[
            {
                "name": "exp_id",
                "required": True,
                "description": "The ID of the experiment, whose config will be written.",
                "paramType": "path",
                "dataType": basestring.__name__
            },
            {
                "name": "base64",
                "description": "The base64 encoded string with the content of the file.",
                "required": True,
                "paramType": "body",
                "dataType": _Conf.__name__
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
        Send a configuration file of an experiment to the server
        (currently, the file will be written with the ending "_new")

        :param path exp_id: The experiment ID
        :<json body json string filename: Name of the experiment file) (currently not used)
        :<json body json string base64: Contents of the file encoded as base64
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 500: Error saving file
        :status 404: The experiment with the given ID was not found
        :status 400: The given base64 has an error
        :status 200: Success. File written.
        """

        body = request.get_json(force=True)
        encoded_data = body['base64']

        filename = get_experiment_conf(exp_id)

        save_file(encoded_data, filename)
        return {'message': "Success. File written: '{0}'".format(filename)}, 200
