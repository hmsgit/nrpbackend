"""
This module contains the REST implementation
for loading (and someday writing) experiment preview images
"""


__author__ = 'Bernd Eckstein'

from flask import Response
from flask_restful import Resource
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import get_experiments, get_basepath

import os

# pylint: disable=R0201


class ExperimentPreview(Resource):
    """
    Get Experiment preview PNG file as octet-stream
    """

    @swagger.operation(
        notes='Get the preview image of a given experiment',
        type='png file',
        produces='application/octet-stream',
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
                "code": 501,
                "message": "Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty"
            },
            {
                "code": 404,
                "message": "The experiment was not found"
            },
            {
                "code": 401,
                "message": "The experiment has no preview image"
            },
            {
                "code": 200,
                "message": "Success. The preview image was sent."
            }
        ]
    )
    def get(self, exp_id):
        """
        Gets preview image of the experiment specified with experiment ID.

        :param exp_id: The experiment ID
        :return Flask.Response mimetype='application/octet-stream' containing preview image
        :status 501: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The experiment with the given ID was not found
        :status 401: The experiment has no preview image
        :status 200: The preview image is returned
        """

        # Check experiment
        experiment_dict = get_experiments()
        if not exp_id in experiment_dict:
            raise NRPServicesClientErrorException("The experiment with the given ID was not "
                                                  "found", 404)

        # Get Experiment
        experiment_file = experiment_dict[exp_id]['experimentConfiguration']
        preview_file = os.path.join(get_basepath(), os.path.splitext(
            experiment_file)[0] + ".png")

        if not os.path.isfile(preview_file):
            raise NRPServicesClientErrorException("The experiment has no preview image.", 401)

        with open(preview_file, "rb") as _file:
            data = _file.read()

        resp = Response(data, status=200, mimetype='application/octet-stream')
        return resp
