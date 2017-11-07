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
This module contains the REST call to clone an experiment
from the template experiments to the storage
"""

__author__ = 'Manos Angelidis'

import logging

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

# Import data base models
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.__UserAuthentication import UserAuthentication

logger = logging.getLogger(__name__)

# pylint: disable=no-self-use


class ExperimentClone(Resource):
    """
    The resource used to clone experiments to the storage
    """

    @swagger.model
    class _ExperimentCloner(object):
        """
        On client call, we flatten the experiment related files, and store them
        persistently to the storage.
        Only used for swagger documentation
        """
        resource_fields = {
            'envPath': fields.String(),
            'brainPath': fields.String(),
            'robotPath': fields.String()
        }
        required = []

    @swagger.operation(
        notes='Clones a template experiment to the storage',
        responseClass=_ExperimentCloner.__name__,
        parameters=[
            {
                "name": "context_id",
                "description": "The context id of the collab if we are in collab mode",
                "required": False,
                "paramType": "body",
                "dataType": str.__name__
            },
            {
                "name": "exp_configuration_path",
                "description": "The path to the experiment configuration file to clone",
                "required": True,
                "paramType": "body",
                "dataType": str.__name__
            },
            {
                "name": "models_paths",
                "description": "the paths to the models files",
                "paramType": "body",
                "required": False,
                "dataType": _ExperimentCloner.__name__
            }
        ],
        responseMessages=[
            {
                "code": 400,
                "message": "No experimentID given"
            },
            {
                "code": 200,
                "message": "Experiment cloned successfully."
            }
        ]
    )
    def put(self):
        """
        Clones a template experiment to the storage
        :< dict models_paths: The object containing paths to the environment, brain and robot.

        :status 400: No experiment ID given.
        :status 200: Success. The experiment was successfully cloned
        """
        # pylint: disable=no-member
        from hbp_nrp_backend.storage_client_api.StorageClient \
            import StorageClient

        client = StorageClient()

        body = request.get_json(force=True)
        if 'exp_configuration_path' not in body:
            raise NRPServicesClientErrorException(
                "No Experiment Configuration path given!")
        context_id = None
        if 'context_id' in body:
            context_id = body['context_id']

        exp_path = body['exp_configuration_path']

        client.clone_experiment_template_to_storage(
            UserAuthentication.get_header_token(request),
            exp_path,
            paths=None,
            context_id=context_id)

        return {"message": "Success. Experiment Cloned Successfully"}, 200
