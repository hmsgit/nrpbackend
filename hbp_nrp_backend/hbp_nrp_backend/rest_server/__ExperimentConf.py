"""
This module contains the REST implementation
for loading and saving experiment configuration files
"""


__author__ = 'Bernd Eckstein'

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import save_file
from hbp_nrp_backend.rest_server.__ExperimentService import get_experiments, get_basepath
#from hbp_nrp_backend.exd_config.generated import generated_experiment_api
#from hbp_nrp_cle.bibi_config import bibi_configuration_script
#import hbp_nrp_backend.exd_config.generated.generated_bibi_api as bibi


import os
import base64

# pylint: disable=R0201
# becaus it seems to be buggy:
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
                "description": "The ID of the experiment retrieved",
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
                "message": "The experiment file was not found"
            },
            {
                "code": 200,
                "message": "Success. The state of the simulation with the given ID is retrieved"
            }
        ]
    )
    def get(self, exp_id):
        """
        Gets configuration file of the experiment specified with experiment ID.

        :param exp_id: The experiment ID
        :>json string filename: Name of the experiment file
        :>json string base64: Contents of the file encoded as base64
        :status 501: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The experiment with the given ID was not found
        :status 401: The experiment file was not found
        :status 200: The data of the file is returned
        """

        # Check experiment
        experiment_dict = get_experiments()
        if not exp_id in experiment_dict:
            raise NRPServicesClientErrorException("The experiment with the given ID was not "
                                                  "found", 404)

        # Get Experiment
        experiment_file = experiment_dict[exp_id]['experimentConfiguration']
        experiment_conf = os.path.join(get_basepath(), experiment_file)

        """
        ###TESTS HERE
        ### Get BIBI:
        experiment = generated_experiment_api.parse(experiment_conf, silence=True)
        bibi_conf = os.path.join(get_basepath(experiment_conf),
                                 experiment.get_bibiConf())
        #transferfunction = os.path.join(get_basepath(experiment_conf), experiment.bibiConf)
        #statemachines = os.path.join(get_basepath(experiment_conf), experiment.bibiConf)

        bibi_api = bibi.parse(bibi_conf, silence=True)
        assert isinstance(bibi_api, bibi.BIBIConfiguration)
        brain_file = bibi_api.brainModel.get_file()

        ### TEST
        print("BIBI: '{0}'".format(bibi_conf))
        print(" - Brain: '{0}'".format(brain_file))
        """

        if not os.path.isfile(experiment_conf):
            raise NRPServicesClientErrorException("The experiment file was not found.", 401)

        with open(experiment_conf, "rb") as _file:
            data = base64.b64encode(_file.read())

        return {'filename': experiment_conf, 'base64': data}, 200

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
                "code": 501,
                "message": "Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty"
            },
            {
                "code": 404,
                "message": "The experiment was not found"
            },
            {
                "code": 402,
                "message": "Error saving file"
            },
            {
                "code": 401,
                "message": "Error in base64: [error_message]"
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
        (currently, the file will be written with the ending "_new"

        :param path exp_id: The experiment ID
        :param body json string filename: Name of the experiment file) (currently not used)
        :param body json string base64: Contents of the file encoded as base64
        :returns
        :status 410: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The experiment with the given ID was not found
        :status 402: Error saving file
        :status 401: The given base64 has an error
        :status 200: Success. File written.
        """

        body = request.get_json(force=True)
        encoded_data = body['base64']

        # Check Experiment
        experiment_dict = get_experiments()
        if not exp_id in experiment_dict:
            raise NRPServicesClientErrorException("The experiment with the given ID was not "
                                                  "found", 404)

        # Get Experiment
        experiment_file = experiment_dict[exp_id]['experimentConfiguration']
        experiment_conf = os.path.join(get_basepath(), experiment_file)

        if save_file(encoded_data, experiment_conf):
            return {'message': "Success. File written: '{0}'".format(experiment_file)}, 200
        else:
            return {'message': "Error saving file."}, 402
