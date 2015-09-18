"""
This module contains the REST implementation
for loading and saving experiment transfer functions
"""


__author__ = 'Bernd Eckstein'

import os

from flask_restful import Resource, request, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages, get_bibi_file, \
    get_transfer_functions, substitute_bibi_transferfunctions


# pylint: disable=no-self-use


@swagger.model
class TransferFunctionDictionary(object):
    """
    Swagger documentation object
    TransferFunctionDict ... tried to make it look like a dictionary for the swagger doc
    """
    resource_fields = {
        'tf_id_1': str.__name__,
        'tf_id_2': str.__name__,
        'tf_id_n': str.__name__
    }


@swagger.model
@swagger.nested(data=TransferFunctionDictionary.__name__)
class TransferFunctionData(object):
    """
    Swagger documentation object
    Main Data Attribute for parsing convenience on the front-end side.
    """
    resource_fields = {
        'data': fields.Nested(TransferFunctionDictionary.resource_fields)
    }
    required = ['data']


class ExperimentTransferfunctions(Resource):
    """
    The resource to load and save experiment transferfunction files
    """

    @swagger.operation(
        notes='Get all transfer functions of an experiment from the server.',
        responseClass=TransferFunctionData.__name__,
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
                "code": 200,
                "data": "Dictionary with {'tf_id_1': 'tf_id_1_source', ...}"
            }
        ]
    )
    def get(self, exp_id):
        """
        Get all transfer functions of an experiment from the server.

        :param path exp_id: The experiment ID
        :>json body with {data: {'tf_id_1': 'tf_id_1_source', ...}}
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The experiment with the given ID was not found
        :status 200: Success. The transfer functions where returned.
        """

        filename = get_bibi_file(exp_id)
        tf = get_transfer_functions(filename)
        return {'data': tf}, 200

    @swagger.model
    class _TransferFunction(object):
        """
        The source code of a transfer function in a simple string.
        """
        resource_fields = {
            'transfer_functions': fields.List(fields.String)
        }
        required = ['transfer_functions']

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
        notes='Send transfer functions of an experiment to the server. Returns a modified bibi '
              'which includes the sent transfer functions',
        parameters=[
            {
                "name": "transfer_functions",
                "required": True,
                "description": "List of Transferfunctions in Python (list of strings)",
                "paramType": "body",
                "dataType": _TransferFunction.__name__
            },
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
                "code": 200,
                "data": "The new BIBI file",
                "filename": "filename of the BIBI file"
            }
        ]
    )
    def put(self, exp_id):
        """
        Send transfer functions of an experiment to the server. Returns a modified bibi \
        which includes the sent transfer functions.

        :param path exp_id: The experiment ID
        :<json body json array of string transfer_functions: the transfer functions as python
        :>json body json string filename: Name of the BIBI file
        :>json body json string data: Contents of the BIBI file
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 500: Error saving file
        :status 404: The experiment with the given ID was not found
        :status 200: Success. File written.
        """

        body = request.get_json(force=True)
        tf = body['transfer_functions']

        filename = get_bibi_file(exp_id)

        bibi = substitute_bibi_transferfunctions(filename, tf)
        return {'data': bibi, 'filename': os.path.basename(filename)}, 200
