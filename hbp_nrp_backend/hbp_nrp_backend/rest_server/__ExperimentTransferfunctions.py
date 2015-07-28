"""
This module contains the REST implementation
for loading and saving experiment transfer functions
"""


__author__ = 'Bernd Eckstein'

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger


from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages, get_bibi_file, \
    substitute_bibi_transferfunctions

import os

# pylint: disable=no-self-use


class ExperimentTransferfunctions(Resource):
    """
    The resource to load and save experiment transferfunction files
    """

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
