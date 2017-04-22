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
for retrieving the transfer
functions used during an experiment.
"""

import logging
import re
from flask_restful_swagger import swagger
from flask_restful import Resource, fields
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort

__author__ = 'LucGuyot, DanielPeppicelli'

# pylint: disable=no-self-use
# We need to define the methods like "get", "put", "post", ... in
# the class even though they could be static. This is because of the
# Swagger framework.

logger = logging.getLogger(__name__)


def get_tf_name(source):
    """
    Get the transfer function def name from its python source code.

    :param source: The source code of the transfer function
    :return: transfer function name
    """
    matches = re.findall(r"def\s+(\w+)\s*\(", source)
    return matches[0] if matches else None


@swagger.model
class TransferFunctionDictionary(object):
    """
    Swagger documentation object
    TransferFunctionDict ... tried to make it look like a dictionary for the swagger doc
    """
    resource_fields = {
        'tf_id_1': str.__name__,
        'tf_id_2': str.__name__,
        'tf_id_n': str.__name__
    }


@swagger.model
@swagger.nested(data=TransferFunctionDictionary.__name__)
class TransferFunctionData(object):
    """
    Swagger documentation object
    Main Data Attribute for parsing convenience on the front-end side.
    """
    resource_fields = {
        'data': fields.Nested(TransferFunctionDictionary.resource_fields)
    }
    required = ['data']


class SimulationTransferFunctions(Resource):
    """
    Expose the source code of the CLE transfer functions as a REST service.
    """

    def __init__(self):
        Resource.__init__(self)

    @swagger.operation(
        notes='Gets all transfer functions',
        responseClass=TransferFunctionData.__name__,
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 200,
                "message": "Transfer functions retrieved successfully"
            }
        ]
    )
    def get(self, sim_id):
        """
        Gets all transfer functions
        (robot to neuron and neuron to robot) in a dictionary with string values.

        :param sim_id: The simulation ID whose transfer functions are retrieved
        :>json dict data: Dictionary containing all transfer functions ('name': 'source')
        :status 404: The simulation with the given ID was not found
        :status 200: Transfer functions retrieved successfully
        """

        simulation = _get_simulation_or_abort(sim_id)

        transfer_functions = dict()
        transfer_functions_list = simulation.cle.get_simulation_transfer_functions()
        for tf in transfer_functions_list:
            name = get_tf_name(tf)
            transfer_functions[name] = tf

        return dict(data=transfer_functions), 200
