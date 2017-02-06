"""
This module contains the REST implementation
for loading and saving experiment brain files
"""

__author__ = 'Bernd Eckstein'

from flask_restful import Resource, fields
from flask_restful_swagger import swagger
from flask import request

from hbp_nrp_backend.rest_server.__ExperimentService import \
    ErrorMessages
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server import NRPServicesWrongUserException
from hbp_nrp_backend.rest_server.__UserAuthentication import \
    UserAuthentication

import json

# pylint: disable=no-self-use
# because it seems to be buggy:
# pylint: disable=pointless-string-statement


class SimulationBrainFile(Resource):
    """
    The resource to get and set brain files in the running simulation
    """

    @swagger.model
    class _brain_(object):
        """
        Get and Set Experiment brain
        Only used for swagger documentation
        """

        resource_fields = {
            'brain_type': fields.String(),
            'data_type': fields.String(),
            'data': fields.String()
        }
        required = ['brain_type', 'data_type', 'data']

    class _error(object):
        """
        Get and Set Experiment brain
        Only used for swagger documentation
        """

        resource_fields = {
            'error_message': fields.String(),
            'error_line': fields.String(),
            'error_column': fields.String()
        }
        required = []

    @swagger.operation(
        notes='Get the brain file of the given simulation.',
        responseClass=_brain_.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation",
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
                "message": "The simulation with the given ID was not found"
            },
            {
                "code": 200,
                "message": "Success. The brain file was retrieved"
            }
        ]
    )
    def get(self, sim_id):
        """
        Get brain data of the simulation specified with simulation ID.

        :param sim_id: The simulation ID
        :>json string brain_type: Type of the brain file ('h5' or 'py')
        :>json string data_type: type of the data field ('text' or 'base64')
        :>json string data: Contents of the brain file. Encoding given in field data_type
        :>json brain_populations:  A dictionary indexed by population names and
        containing neuron indices. Neuron indices could be defined by individual integers,
        lists of integers or python slices. Python slices are defined by a
        dictionary containing the 'from', 'to' and 'step' values.
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The simulation with the given ID was not found
        :status 200: Success. The experiment brain file was retrieved
        """

        simulation = _get_simulation_or_abort(sim_id)

        result = simulation.cle.get_simulation_brain()

        return {
            'data': result.brain_data,
            'brain_type': result.brain_type,
            'data_type': result.data_type,
            'additional_populations': json.loads(result.brain_populations)
        }, 200

    @swagger.operation(
        notes='Get the brain file of the given simulation.',
        responseClass=_error.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation",
                "required": True,
                "paramType": "path",
                "dataType": basestring.__name__
            },
            {
                "name": "data",
                "description": "The brain data",
                "required": True,
                "paramType": "body",
                "dataType": _brain_.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.MODEXP_VARIABLE_ERROR
            },
            {
                "code": 404,
                "message": "The simulation with the given ID was not found"
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 300,
                "message": "Failed. Error found in given brain file"
            },
            {
                "code": 200,
                "message": "Success. The brain file was replaced"
            }
        ]
    )
    def put(self, sim_id):
        """
        Set brain file of the simulation specified with simulation ID.
        Depending on the type of brain file, it has to be transmitted as text or
        as base64 (given in the field data_type)

        :param sim_id: The simulation ID
        :<json string brain_type: Type of the brain file ('h5' or 'py')
        :<json string data_type: type of the data field ('text' or 'base64')
        :<json string data: Contents of the brain file. Encoding given in field data_type
        :<json brain_populations:  A dictionary indexed by body['additional_populations'] names
        and containing neuron indices. Neuron indices could be defined by individual integers,
        lists of integers or python slices. Python slices are defined by a
        dictionary containing the 'from', 'to' and 'step' values.
        :>json string error_message: Error Message if there is a syntax error in the code
        :>json int error_line: Line of code, where error occurred
        :>json int error_column: Column, where error occurred (if available)
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The simulation with the given ID was not found
        :status 401: Operation only allowed by simulation owner
        :status 300: Error in given brain file
        :status 200: Success. The experiment brain file was replaced
        """

        simulation = _get_simulation_or_abort(sim_id)
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        body = request.get_json(force=True)
        result = simulation.cle.set_simulation_brain(body['brain_type'],
                                                     body['data'],
                                                     body['data_type'],
                                                     json.dumps(body['additional_populations']),
                                                     body['change_population'])

        if result.error_message is not "":
            # Error in given brain
            return {'error_message': result.error_message,
                    'error_line': result.error_line,
                    'error_column': result.error_column,
                    'handle_population_change': result.handle_population_change}, 300

        # Success
        return {'message': "Success"}, 200
