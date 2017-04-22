# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
This module contains the REST implementation
for setting the source code of a transfer
used in an experiment.
"""

import logging
from flask import request
from flask_restful_swagger import swagger
from flask_restful import Resource
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server import NRPServicesTransferFunctionException, \
    NRPServicesWrongUserException
from hbp_nrp_backend.rest_server.__UserAuthentication import \
    UserAuthentication

__author__ = 'LucGuyot, DanielPeppicelli'

# pylint: disable=no-self-use
# We need to define the methods like "get", "put", "post", ... in
# the class even though they could be static. This is because of the
# Swagger framework.

logger = logging.getLogger(__name__)


class SimulationTransferFunction(Resource):
    """
    REST service for patching transfer function source code.
    """

    def __init__(self):
        Resource.__init__(self)

    @swagger.operation(
        notes='Applies user changes to transfer function code.',
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
                "name": "transfer_function_name",
                "description": "The original name of the transfer function",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
            {
                "name": "source",
                "description": "The transfer function source code to patch",
                "required": True,
                "paramType": "body",
                "dataType": str.__name__
            }
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 400,
                "message": "The source code is invalid"
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 200,
                "message": "Success. The code was successfully patched"
            }
        ]
    )
    def put(self, sim_id, transfer_function_name):
        """
        Applies user changes to transfer function code

        :param sim_id: The simulation id
        :param transfer_function_name: The transfer function original name
        :status 400: The source code is invalid
        :status 401: Insufficient permissions to apply changes
        :status 404: The simulation with the given ID was not found
        :status 200: The code was successfully patched
        """
        simulation = _get_simulation_or_abort(sim_id)
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        transfer_function_source = request.data
        error_message = simulation.cle.set_simulation_transfer_function(
            transfer_function_name,
            transfer_function_source
        )
        if error_message:
            raise NRPServicesTransferFunctionException(
                "Transfer function patch failed: "
                + error_message + "\n"
                + "Updated source:\n"
                + str(transfer_function_source)
            )
        return 200

    @swagger.operation(
        notes='Delete a transfer function.',
        responseClass=int.__name__,
        parameters=[
            {
                "name": "sim_id",
                "required": True,
                "description": "The ID of the simulation whose transfer function will be deleted",
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "transfer_function_name",
                "description": "The name of the transfer function to delete",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 200,
                "message": "Success. The delete operation was successfully called. This "
                           "does not imply that the transfer function was correctly "
                           "deleted though."
            }
        ]
    )
    def delete(self, sim_id, transfer_function_name):
        """
        Delete a transfer function

        :param sim_id: The simulation id
        :param transfer_function_name: The name of the transfer function to delete
        :status 401: Insufficient permissions to apply changes
        :status 404: The simulation with the given ID was not found
        :status 200: The delete operation was successfully called
        """
        simulation = _get_simulation_or_abort(sim_id)
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        response = simulation.cle.delete_simulation_transfer_function(
            transfer_function_name
        )

        if response is False:
            raise NRPServicesTransferFunctionException(
                "Transfer function delete failed: " + str(transfer_function_name))
        return 200
