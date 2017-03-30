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
for loading and saving experiment brain files
"""


__author__ = 'Bernd Eckstein'

import os
import tempfile
import shutil
import logging
from threading import Thread

from flask_restful import Resource, fields, request
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import \
    ErrorMessages
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_commons.generated import bibi_api_gen

logger = logging.getLogger(__name__)

# pylint: disable=R0201
# because it seems to be buggy:
# pylint: disable=W0105


class ExperimentBrainFile(Resource):
    """
    The resource to save experiment brain files to the collab
    """

    def parsePopulations(self, brain_populations, bibi):
        """
        Parse a population from the received brain_populations object and add it to bibi.

        """

        if isinstance(brain_populations, list):
            for value in brain_populations:
                population_node = None
                if isinstance(value, dict):
                    population_node = bibi_api_gen.Range()
                    population_node.from_ = value['from']
                    population_node.to = value['to']
                    population_node.step = value.get('step')
                    if population_node.step <= 0:
                        population_node.step = 1
                    population_node.population = value['name']
                    bibi.brainModel.populations.append(population_node)
        else:
            for key, value in brain_populations.iteritems():
                population_node = None
                if isinstance(value, list):
                    population_node = bibi_api_gen.List()
                    for index in value:
                        population_node.append(index)
                if isinstance(value, dict):
                    population_node = bibi_api_gen.Range()
                    population_node.from_ = value['from']
                    population_node.to = value['to']
                    population_node.step = value.get('step')
                    if population_node.step <= 0:
                        population_node.step = 1
                population_node.population = key
                bibi.brainModel.populations.append(population_node)

    @swagger.model
    class _Brain(object):
        """
        Set Experiment brain
        Only used for swagger documentation
        """

        resource_fields = {
            'data': fields.String(),
            'additional_populations': fields.Arbitrary()
        }
        required = ['data']

    @swagger.operation(
        notes='Save a brain model PyNN of an experiment to the collab.',
        parameters=[
            {
                "name": "brain_model",
                "required": True,
                "description": "Brain model in the Python language (using PyNN)",
                "paramType": "body",
                "dataType": _Brain.__name__
            },
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.ERROR_SAVING_FILE_500
            },
            {
                "code": 404,
                "message": ErrorMessages.COLLAB_NOT_FOUND_404
            },
            {
                "code": 400,
                "message": "Neural network python code should be sent in "
                           "the body under the 'data' key"
            },
            {
                "code": 200,
            }
        ]
    )
    def put(self, context_id):
        """
         Save a brain model PyNN of an experiment to the collab.

        :param path context_id: The context UUID of the Collab where the transfer functions
         will be saved
        :<json body json string data: PyNN script of the model
        :<json body json string brain_populations: neuron populations
        :status 500: Error saving file
        :status 404: The collab with the given context ID was not found
        :status 400: The request body is malformed
        :status 200: Success. File written.
        """

        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient

        body = request.get_json(force=True)
        if 'data' not in body:
            raise NRPServicesClientErrorException(
                "Neural network python code should be sent in the body under the 'data' key"
            )

        data = body['data']
        brain_populations = body.get('additional_populations')

        client = NeuroroboticsCollabClient(
            UserAuthentication.get_header_token(request),
            context_id
        )

        replace_brain = Thread(target=client.replace_file_content_in_collab,
                               kwargs={'content': data,
                                       'mimetype': client.BRAIN_PYNN_MIMETYPE})
        replace_brain.start()
        bibi, bibi_file_path, bibi_uuid = client.clone_bibi_file_from_collab_context()
        # Remove all populations from BIBI.
        del bibi.brainModel.populations[:]

        if brain_populations is not None:
            self.parsePopulations(brain_populations, bibi)

        if tempfile.gettempdir() in bibi_file_path:
            logger.debug(
                "removing the temporary bibi configuration file %s",
                bibi_file_path
            )
            shutil.rmtree(os.path.dirname(bibi_file_path))
        client.replace_file_content_in_collab(
            bibi.toxml("utf-8"),
            bibi_uuid
        )
        replace_brain.join()
        return 200
