"""
This module contains the REST services to setup the simulation
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server import api
from hbp_nrp_backend.rest_server.__SimulationControl import \
    SimulationControl
from hbp_nrp_backend.rest_server.__UserAuthentication import \
    UserAuthentication
from flask import request
from flask_restful import Resource, marshal_with, fields
from flask_restful_swagger import swagger

# pylint: disable=R0201


class SimulationService(Resource):
    """
    The module to setup simulations
    """
    @swagger.model
    class _ExperimentID(object):
        """
        Represents a simulation ID object. Only used for swagger documentation
        """

        resource_fields = {
            'experimentID': fields.String()
        }
        required = ['experimentID']

    @swagger.operation(
        notes='This is the entry point for the NRP REST server since'
              ' this is where you actually create simulations',
        responseClass=Simulation.__name__,
        responseMessages=[
            {
                "code": 400,
                "message": "Experiment ID is not valid"
            },
            {
                "code": 201,
                "message": "Simulation created successfully"
            }
        ],
        parameters=[
            {
                "name": "body",
                "paramType": "body",
                "dataType": _ExperimentID.__name__,
                "required": True
            }
        ]
    )
    @marshal_with(Simulation.resource_fields)
    def post(self):
        """
        Creates a new simulation. The simulation is not started
        """
        body = request.get_json(force=True)
        sim_id = len(simulations)
        if 'experimentID' in body:
            sim_owner = UserAuthentication.get_x_user_name_header(request)
            simulations.append(Simulation(sim_id, body['experimentID'], sim_owner))
        else:
            return None, 400
        return simulations[sim_id], 201, \
               {'location': api.url_for(SimulationControl, sim_id=sim_id)}

    @swagger.operation(
        notes='Gets the list of all simulations on the server,'
              ' no matter what state',
        responseClass=list.__name__,
        responseMessages=[
            {
                "code": 200,
                "message": "Simulations retrieved successfully"
            }
        ]
    )
    @marshal_with(Simulation.resource_fields)
    def get(self):
        """
        Gets the list of simulations
        """
        return simulations, 200
