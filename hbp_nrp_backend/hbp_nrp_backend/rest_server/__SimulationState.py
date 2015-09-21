"""
This module contains the REST implementation for the control of the simulation state
"""

__author__ = 'GeorgHinkel'

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.simulation_control import InvalidStateTransitionException
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server import NRPServicesStateException, \
    NRPServicesWrongUserException
from hbp_nrp_backend.rest_server.__UserAuthentication import \
    UserAuthentication

# pylint: disable=R0201


class SimulationState(Resource):
    """
    The resource to control the state of the simulation.
    Allowed state values are: created, initialized, started, paused, stopped
    """

    @swagger.model
    class _State(object):
        """
        State of a simulation. Allowed values are: created, initialized, started,
        paused, stopped.
        Only used for swagger documentation
        """

        resource_fields = {
            'state': fields.String()
        }
        required = ['state']

    @swagger.operation(
        notes='Gets the state of the given simulation.'
        ' Possible values are: created, initialized, started, paused, stopped',
        responseClass=_State.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "required": True,
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
                "message": "Success. The state of the simulation with the given ID is retrieved"
            }
        ]
    )
    def get(self, sim_id):
        """
        Gets the state of the simulation with the specified simulation id. Possible values are:
        created, initialized, started, paused, stopped

        :param sim_id: The simulation id
        :>json string state: The state of the simulation
        :>json string timeout: The timeout set for the simulation
        :status 404: The simulation with the given ID was not found
        :status 200: The state of the simulation with the given ID was successfully retrieved
        """
        simulation = _get_simulation_or_abort(sim_id)
        return {'state': str(simulation.state)}, 200

    @swagger.operation(
        notes='Sets the state of the given simulation.'
        ' Allowed values are: created, initialized, started, paused, stopped',
        responseClass=_State.__name__,
        parameters=[
            {
                "name": "sim_id",
                "required": True,
                "description": "The ID of the simulation whose state shall be set",
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "state",
                "description": "The new state of the simulation",
                "required": True,
                "paramType": "body",
                "dataType": _State.__name__
            }
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 400,
                "message": "The state transition is invalid"
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 200,
                "message": "Success. The new state has been correctly applied"
            }
        ]
    )
    def put(self, sim_id):
        """
        Sets the simulation with the given name into a new state. Allowed values are:
        created, initialized, started, paused, stopped

        :param sim_id: The simulation id
        :<json string state: The state of the simulation to set
        :>json string state: The state of the simulation
        :>json string timeout: The timeout set for the simulation
        :status 400: The state transition is invalid
        :status 401: Operation only allowed by simulation owner
        :status 404: The simulation with the given ID was not found
        :status 200: The state of the simulation with the given ID was successfully set
        """
        simulation = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        body = request.get_json(force=True)
        try:
            simulation.state = body['state']
        except InvalidStateTransitionException:
            raise NRPServicesStateException(
                "Invalid transition (" + simulation.state + "->" + body['state'] + ")")
        return {'state': str(simulation.state)}, 200
