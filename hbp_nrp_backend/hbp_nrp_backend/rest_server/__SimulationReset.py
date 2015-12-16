"""
This module contains REST services for handling simulations reset.
"""

__author__ = "Alessandro Ambrosano"

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException

from hbp_nrp_backend.rest_server import NRPServicesGeneralException, \
    NRPServicesWrongUserException, NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication

# pylint: disable=no-self-use


class SimulationReset(Resource):
    """
    This resource handles the reset of a simulation, forwarding all the reset requests to the
    respective CLE instance.
    """

    @swagger.model
    class ResetRequest(object):
        """
        Represents a request for the API implemented by SimulationReset
        """

        resource_fields = {
            'oldReset': fields.Boolean,
            'robotPose': fields.Boolean,
            'fullReset': fields.Boolean
        }
        required = ['oldReset', 'robotPose', 'fullReset']

    @swagger.operation(
        notes='Handles the reset of a given simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "body",
                "paramType": "body",
                "dataType": ResetRequest.__name__,
                "required": True
            }
        ],
        responseMessages=[
            {
                "code": 200,
                "message": "Success. The reset type requested was performed on the given "
                    "simulation."
            },
            {
                "code": 400,
                "message": "Invalid request, the JSON parameters are incorrect."
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 404,
                "message": "The simulation was not found."
            },
            {
                "code": 500,
                "message": "Reset unsuccessful due to a server error, better specified by the"
                    "error message."
            }
        ]
    )
    def put(self, sim_id):
        """
        Calls the CLE for resetting a given simulation.

        :param sim_id: The simulation ID.
        :>json oldReset: this reset can be performed directly by the frontend, and will be
            eventually removed.
        :>json robotPose: request to reset the robot pose to the initial one.
        :>json fullReset: request to reset completely the simulation.
        :status 200: The requested reset was performed successfully.
        :status 400: Invalid request, the JSON parameters are incorrect.
        :status 401: Operation only allowed by simulation owner.
        :status 404: The simulation with the given ID was not found.
        :status 500: Reset unsuccessful due to a server error, better specified by the error
            message.
        """

        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, sim.owner):
            raise NRPServicesWrongUserException()

        body = request.get_json(force=True)

        for par in SimulationReset.ResetRequest.required:
            if par not in body:
                raise NRPServicesClientErrorException('Missing parameter %s' % (par, ))

        for par in body:
            if par not in SimulationReset.ResetRequest.resource_fields:
                raise NRPServicesClientErrorException('Invalid parameter %s' % (par, ))

        try:
            sim.cle.reset(
                reset_robot_pose=body.get('robotPose'),
                full_reset=body.get('fullReset')
            )
        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        return {}, 200
