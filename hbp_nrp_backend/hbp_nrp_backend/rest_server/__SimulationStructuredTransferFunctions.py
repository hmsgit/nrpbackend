"""
This module contains the Flask resource for structured transfer functions
"""

import re
import json
from flask import request
from flask_restful_swagger import swagger
from flask_restful import Resource, fields
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server import NRPServicesTransferFunctionException, \
    NRPServicesWrongUserException
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication


__author__ = "Georg Hinkel"


def get_tf_name(source):
    """
    Get the transfer function def name from its python source code.

    :param source: The source code of the transfer function
    :return: transfer function name
    """
    matches = re.findall(r"def\s+(\w+)\s*\(", source)
    return matches[0] if matches else None


class JSONObject(object):
    """
    A helper class
    """

    def __init__(self, d):
        """
        Creates a new JSON object with the given dict

        :param d: The dictionary for the object
        """
        self.__dict__ = d


@swagger.model
class NeuronModel(object):
    """
    Swagger documentation object
    """
    resource_fields = {
        'name': fields.String(),
        'type': fields.Integer(),
        'ids': fields.Nested(int.__name__),
        'start': fields.Integer(),
        'stop': fields.Integer(),
        'step': fields.Integer()
    }
    required = ['name', 'type']


@swagger.model
@swagger.nested(data=NeuronModel.__name__)
class DeviceModel(object):
    """
    Swagger documentation object
    """
    resource_fields = {
        'name': fields.String(),
        'type': int.__name__,
        'neurons': fields.Nested(NeuronModel.__name__)
    }
    required = ['name', 'type', 'neurons']


@swagger.model
class TopicModel(object):
    """
    Swagger documentation object
    """
    resource_fields = {
        'name': fields.String(),
        'topic': fields.String(),
        'type': fields.String(),
        'publishing': fields.Boolean()
    }
    required = ['']


@swagger.model
class VariableModel(object):
    """
    Swagger documentation object
    """
    resource_fields = {
        'name': fields.String(),
        'type': fields.String(),
        'initial value': fields.String()
    }
    required = []


@swagger.model
@swagger.nested(data=DeviceModel.__name__)
class TransferFunctionModel(object):
    """
    Swagger documentation object
    """
    resource_fields = {
        'name': fields.String(),
        'code': fields.String(),
        'type': fields.Integer(),
        'devices': fields.Nested(DeviceModel.__name__),
        'topics': fields.Nested(TopicModel.__name__),
        'variables': fields.Nested(VariableModel.__name__)
    }


@swagger.model
@swagger.nested(data=TransferFunctionModel.__name__)
class TransferFunctionsData(object):
    """
    Swagger documentation object
    """
    resource_fields = {
        'transferFunctions': fields.Nested(TransferFunctionModel.resource_fields)
    }
    required = ['transferFunctions']


# pylint: disable=no-self-use
class SimulationStructuredTransferFunctions(Resource):
    """
    The resource for structured transfer functions
    """

    @swagger.operation(
        notes='Get transfer functions in a structured format',
        responseClass=TransferFunctionsData.__name__,
        parameters=[
            {
                "name": "sim_id",
                "required": True,
                "description": "The ID of the simulation whose transfer functions are obtained",
                "paramType": "path",
                "dataType": int.__name__
            }
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 200,
                "message": "Success. The transfer functions were returned"
            }
        ]
    )
    def get(self, sim_id):
        """
        Gets the structured transfer functions

        :param sim_id: The simulation id
        :return: A list of structured transfer functions
        """
        simulation = _get_simulation_or_abort(sim_id)

        tfs = []
        transfer_functions_list = simulation.cle.get_structured_transfer_functions()
        for tf in transfer_functions_list:
            devices = [{
                'name': dev.name,
                'type': dev.type,
                'neurons': {
                    'name': dev.neurons.name,
                    'type': dev.neurons.type,
                    'ids': [
                        {
                            'id': n_id
                        } for n_id in dev.neurons.ids
                    ],
                    'start': dev.neurons.start,
                    'stop': dev.neurons.stop,
                    'step': dev.neurons.step
                }
            } for dev in tf.devices]

            topics = [{
                'name': top.name,
                'topic': top.topic,
                'type': top.type,
                'publishing': top.publishing
            } for top in tf.topics]

            variables = [{
                'name': var.name,
                'type': var.type,
                'initial_value': var.initial_value
            } for var in tf.variables]

            tf_e = {
                'name': tf.name,
                'code': tf.code,
                'type': tf.type,
                'devices': devices,
                'topics': topics,
                'variables': variables
            }
            tfs.append(tf_e)
        return {'transferFunctions': tfs}, 200

    @swagger.operation(
        notes='Applies user changes to a structured transfer function.',
        responseClass=int.__name__,
        parameters=[
            {
                "name": "sim_id",
                "required": True,
                "description": "The ID of the simulation whose transfer function will be modified",
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "transfer_function",
                "description": "The structured transfer function",
                "required": True,
                "paramType": "body",
                "dataType": TransferFunctionsData.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": "Simulation in state [STATE]. Can't update transfer function."
            },
            {
                "code": 404,
                "message": "The simulation with the given ID was not found"
            },
            {
                "code": 400,
                "message": "The passed transfer function is invalid"
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 200,
                "message": "Success. The transfer function was successfully patched"
            }
        ]
    )
    def put(self, sim_id):
        """
        Puts the structured transfer functions

        :param sim_id: The simulation id
        """
        simulation = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        transfer_function = json.loads(request.data, object_hook=JSONObject)
        error_message = simulation.cle.set_structured_transfer_function(transfer_function)
        if error_message:
            raise NRPServicesTransferFunctionException(
                "Transfer function patch failed: "
                + error_message + "\n"
                + "Transfer function:\n"
                + request.data
            )
        return 200
