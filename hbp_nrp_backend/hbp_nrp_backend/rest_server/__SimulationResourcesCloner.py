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
This module contains the REST implementation
for loading and saving experiment brain files
"""

__author__ = 'Kepa Cantero'

from flask_restful import Resource
from flask_restful_swagger import swagger
from flask import request

from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient
from hbp_nrp_backend.rest_server import ErrorMessages
from hbp_nrp_backend.__UserAuthentication import UserAuthentication


class SimulationResourcesCloner(Resource):
    """
    Clones The files from the experiments resources to tmp/nrpTemp/resources.
    """

    @swagger.operation(
        notes='Save the resources files from the running experiment to tmp folder',
        parameters=[
            {
                "name": "experiment_id",
                "description": "The Id of the selected experiment",
                "required": True,
                "paramType": "body",
                "dataType": str.__name__
            },
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.ERROR_SAVING_FILE_500
            },
            {
                "code": 200,
                "message": "Success. File written."
            }
        ]
    )
    # pylint: disable=no-self-use
    def post(self):
        """
        Clone the resources files from the storage to the tmp folder
        :param experiment_id: The experiment ID
        :status 200: Success. File written.
        """
        body = request.get_json(force=True)
        exp_id = body.get('exp_id', None)
        client = StorageClient()
        try:
            client.copy_resources_folders_to_tmp(UserAuthentication.get_header_token(
                request), exp_id)
            return {'msg': 'The files were copied successfull'}, 200
        except Exception:
            raise NRPServicesGeneralException(
                ErrorMessages.ERROR_SAVING_FILE_500,
                "Server Error: Files were not copied to the tmp folder", 500)
