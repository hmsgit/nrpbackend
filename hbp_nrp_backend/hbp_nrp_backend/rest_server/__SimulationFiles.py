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
This module contains REST services for handling files in a running simulation
"""

import logging

from flask_restful import Resource
from flask_restful_swagger import swagger

from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException

from hbp_nrp_backend import NRPServicesGeneralException, NRPServicesWrongUserException
from hbp_nrp_backend.rest_server import ErrorMessages

from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.__UserAuthentication import UserAuthentication

__author__ = 'Hossain Mahmud'

logger = logging.getLogger(__name__)


class SimulationFiles(Resource):    # pragma: no cover
    """
    This resource handles the files in a running simulation
    """
    RESOURCE_TYPES = ["robots", "brains", "environments", "files"]

    @swagger.operation(
        notes='Handles files in the running simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose files shall be handled",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "resource_type",
                "description": "Type of the resource. Options: robots, brains, environments, files",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
            {
                "name": "resource_path",
                "description": "Path of the resource",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            }
        ],
        responseMessages=[
            {"code": 500, "message": ErrorMessages.SERVER_ERROR_500},
            {"code": 404, "message": ErrorMessages.SIMULATION_NOT_FOUND_404},
            {"code": 401, "message": ErrorMessages.SIMULATION_PERMISSION_401_VIEW},
            {"code": 400, "message": "Invalid request, the JSON parameters are incorrect."},
            {"code": 200, "message": "Success."},
        ]
    )
    def get(self, sim_id, resource_type, resource_path):
        """
        Download the file in the resource_path for the give resource_type
        :param sim_id: The ID of the simulation whose files shall be handled
        :param resource_type: "robots", "brains", "environments", "files"
        :param resource_path: relative path the particular resource to be downloaded
        """
        # pylint: disable=no-self-use
        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.can_view(sim):
            raise NRPServicesWrongUserException()

        if resource_type not in SimulationFiles.RESOURCE_TYPES:
            raise NRPServicesGeneralException("Wrong resource_type specified",
                                              'Wrong Parameter', 500)
        try:
            ret, err = SimulationFiles.__download_file(sim, resource_type, resource_path)
        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        return ({'res': 'success'}, 200) if ret else ({'res': err}, 500)

    @staticmethod
    def __download_file(sim, resource_type, resource_path):
        """
        Download a file at resource_path of type resource_type in the simulation sim.
        """
        return sim.cle.download_file_in_simdir(resource_type, resource_path)
