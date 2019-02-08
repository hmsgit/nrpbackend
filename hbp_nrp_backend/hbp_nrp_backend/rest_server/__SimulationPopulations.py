# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains REST services for getting neurons from a running simulation
"""

__author__ = "Bernd Eckstein"

from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from flask import request

from hbp_nrp_backend import NRPServicesWrongUserException
from hbp_nrp_backend.rest_server import ErrorMessages
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort

from hbp_nrp_commons.bibi_functions import docstring_parameter
from hbp_nrp_backend.__UserAuthentication import UserAuthentication
import json

# pylint: disable=no-self-use


@swagger.model
class NeuronParameter(object):
    """
    NeuronParameter
    Only used for swagger documentation
    """

    resource_fields = {
        'parameterName': fields.String,
        'value': fields.Float,
    }
    required = ['parameterName', 'value']


@swagger.model
@swagger.nested(parameters=NeuronParameter.__name__)
class Population(object):
    """
    Population
    Only used for swagger documentation
    """

    resource_fields = {
        'indices': fields.List(fields.Integer),
        'name': fields.String,
        'neuron_model': fields.String,
        'parameters': fields.List(fields.Nested(NeuronParameter.resource_fields)),
        'gids': fields.List(fields.Integer),
    }
    required = ['indices', 'name', 'neuron_model', 'parameters', 'gids']


@swagger.model
@swagger.nested(populations=Population.__name__)
class Populations(object):
    """
    Populations
    Only used for swagger documentation
    """

    resource_fields = {
        'populations': fields.List(fields.Nested(Population.resource_fields)),
    }
    required = ['populations']


@swagger.model
class SetPopulations(object):
    """
    Set Experiment Brain
    Only used for swagger documentation
    """
    resource_fields = {
        'brain_type': fields.String(),
        'brain_populations': fields.Nested(Population.resource_fields),
        'data_type': fields.String(),
        'change_population': fields.String()
    }
    required = ['brain_type', 'brain_populations',
                'data_type', 'change_population']


class SimulationPopulations(Resource):
    """
    This resource handles getting the populations of a brain in a simulation by retrieving the
    neurons of the respective CLE instance.
    """

    @swagger.operation(
        notes='Gets the neurons of the given simulation.',
        responseClass=Populations.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The simulation ID",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            }
        ],
        responseMessages=[
            {
                "code": 404,
                "message": ErrorMessages.SIMULATION_NOT_FOUND_404
            },
            {
                "code": 200,
                "message": "Success. The population of the brain are retrieved"
            }
        ]
    )
    @docstring_parameter(ErrorMessages.SIMULATION_NOT_FOUND_404)
    def get(self, sim_id):
        """
        Gets the neurons of the brain in a simulation with the specified simulation id.

        :param sim_id: The simulation ID

        :> json array Populations: array of population dictionaries. Each dict contains population
                                   name, neuron indices, neuron model, parameters and gids

        :status 404: {0}
        :status 200: The neurons of the simulation with the given ID where successfully retrieved
        """
        simulation = _get_simulation_or_abort(sim_id)

        # Get Neurons from cle
        neurons = simulation.cle.get_populations()

        return neurons, 200

    @swagger.operation(
        notes='Get the brain file of the given simulation.',
        responseClass=Populations.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The simulation ID",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "data",
                "description": "The population data",
                "required": True,
                "paramType": "body",
                "dataType": SetPopulations.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.MODEXP_VARIABLE_ERROR
            },
            {
                "code": 404,
                "message": ErrorMessages.SIMULATION_NOT_FOUND_404
            },
            {
                "code": 401,
                "message": ErrorMessages.SIMULATION_PERMISSION_401
            },
            {
                "code": 400,
                "message": ErrorMessages.SOURCE_CODE_ERROR_400
            },
            {
                "code": 200,
                "message": "Success. The populations has been updated"
            }
        ]
    )
    @docstring_parameter(ErrorMessages.MODEXP_VARIABLE_ERROR,
                         ErrorMessages.SIMULATION_NOT_FOUND_404,
                         ErrorMessages.SIMULATION_PERMISSION_401,
                         ErrorMessages.SOURCE_CODE_ERROR_400)
    def put(self, sim_id):
        """
        Set the populations of the brain in a simulation with the specified simulation id.

        :param sim_id: The simulation ID

        :< json dict brain_populations: A dictionary indexed by population names and containing
                                             neuron indices

        :< json string brain_type: Type of the brain file ('h5' or 'py')
        :< param brain_populations: Contents of the brain file. Encoding given in field data_type
        :< json string data_type: type of the data field ('text' or 'base64')
        :< json string change_population: indicates if it has to be changes
                                        in the transfer functions
        :> json string message: Error Message if there is a syntax error in the code
        :status 404: {0}
        :status 200: Success. The populations of the brain where successfully set
        """

        simulation = _get_simulation_or_abort(sim_id)
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        body = request.get_json(force=True)

        result = simulation.cle.set_simulation_populations(
            brain_type=body['brain_type'],
            brain_populations=json.dumps(body['brain_populations']),
            data_type=body['data_type'],
            change_population=body['change_population'])

        if result.message:
            return {'error_message': result.message}, 400

        return {'message': 'Success'}, 200
