"""
This module contains the REST implementation
for extending the simulation timeout
"""

from flask import request
from flask_restful import Resource
from flask_restful_swagger import swagger


from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server import NRPServicesWrongUserException
from hbp_nrp_backend.rest_server.__UserAuthentication import \
    UserAuthentication

import datetime
# pylint: disable=R0201


SIMULATION_TIMEOUT_EXTEND = 10  # minutes


class SimulationTimeout(Resource):

    """
    REST service for extending the simulation timeout
    """

    @swagger.operation(
        notes='Extends the simulation timeout',
        parameters=[
            {
                "name": "sim_id",
                "required": True,
                "description": "The ID of the simulation whose timeout shall be extended",
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
                "code": 402,
                "message": "Failed to extend the timeout"
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 200,
                "message": "Success. The simulation timeout has been extended"
            }
        ]
    )
    def post(self, sim_id):
        """
        Extends the simulation timeout

        :param sim_id: The simulation id
        :status 402: Failed to extend the timeout
        :status 401: Operation only allowed by simulation owner
        :status 404: The simulation with the given ID was not found
        :status 200: The simulation timeout has been extended
        """
        simulation = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        new_timeout = simulation.kill_datetime + \
            datetime.timedelta(minutes=SIMULATION_TIMEOUT_EXTEND)

        if not simulation.cle.extend_simulation_timeout(new_timeout):
            return {}, 402

        simulation.kill_datetime = new_timeout

        return {}, 200
