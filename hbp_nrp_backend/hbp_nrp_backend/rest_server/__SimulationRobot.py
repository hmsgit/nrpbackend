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
This module contains REST services for handling simulations robots.
"""

__author__ = 'Hossain Mahmud'

import logging

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException

from hbp_nrp_backend.rest_server import NRPServicesGeneralException, \
    NRPServicesWrongUserException, NRPServicesClientErrorException, ErrorMessages
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.__UserAuthentication import UserAuthentication

logger = logging.getLogger(__name__)


class Parameters(object):
    """
    Constants to be used a parameter names
    """
    ROBOT_ID = 'robotId'
    ROBOT_REL_PATH = 'robotRelPath'
    ROBOT_ABS_PATH = 'robotAbsPath'
    IS_CUSTOM = 'isCustom'


class SimulationRobots(Resource):
    """
    This resource handles the robot models in a running simulation
    """
    @swagger.model
    class RobotRequest(object):
        """
        Represents a request for the API implemented by SimulationRobot
        """
        resource_fields = {
            Parameters.ROBOT_ID: fields.Integer,
            Parameters.ROBOT_REL_PATH: fields.String,
            Parameters.IS_CUSTOM: fields.Boolean
        }

    @swagger.model
    class RobotPutRequest(RobotRequest):
        """
        Lists required fields for a PUT request
        """
        required = [Parameters.ROBOT_ID, Parameters.ROBOT_REL_PATH]

    @swagger.model
    class RobotDelRequest(RobotRequest):
        """
        Lists required fields for a DELETE request
        """
        required = [Parameters.ROBOT_ID]

    @swagger.model
    class RobotGetRequest(RobotRequest):
        """
        Lists required fields for a GET request
        """
        required = []

    @swagger.operation(
        notes='Gets the list of robots in the running simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose robot list shall be retrieved",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "body",
                "paramType": "body",
                "dataType": RobotGetRequest.__name__,
                "required": False
            }
        ],
        responseMessages=[
            {"code": 500, "message": ErrorMessages.SERVER_ERROR_500},
            {"code": 404, "message": ErrorMessages.SIMULATION_NOT_FOUND_404},
            {"code": 401, "message": ErrorMessages.SIMULATION_PERMISSION_401},
            {"code": 400, "message": "Invalid request, the JSON parameters are incorrect."},
            {"code": 200, "message": "Success."},
        ]
    )
    def get(self, sim_id):
        """
        Gets the list of robots in the running simulation.
        """
        # pylint: disable=no-self-use
        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, sim.owner):
            raise NRPServicesWrongUserException()

        try:
            robots = SimulationRobots.__get_simulation_robots(sim)
        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        result = {
            'robots': [{
                           Parameters.ROBOT_ID: robot.robot_id,
                           Parameters.ROBOT_ABS_PATH: robot.robot_model_rel_path,
                           Parameters.IS_CUSTOM: robot.is_custom
                       } for robot in robots
                       ]
        }
        return result, 200

    @swagger.operation(
        notes='Add a new robot to the simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose robot list shall be retrieved",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "body",
                "paramType": "body",
                "dataType": RobotPutRequest.__name__,
                "required": True
            }
        ],
        responseMessages=[
            {"code": 500, "message": ErrorMessages.SERVER_ERROR_500},
            {"code": 404, "message": ErrorMessages.SIMULATION_NOT_FOUND_404},
            {"code": 401, "message": ErrorMessages.SIMULATION_PERMISSION_401},
            {"code": 400, "message": "Invalid request, the JSON parameters are incorrect."},
            {"code": 200, "message": "Success. Robot added."},
        ]
    )
    def put(self, sim_id):
        """
        Add a new robot to the simulation.

        :param sim_id: The simulation ID.

        :status 500: {0}
        :status 404: {1}
        :status 401: {2}
        :status 400: Invalid request, the JSON parameters are incorrect
        :status 200: The requested robot was created successfully
        """
        # pylint: disable=no-self-use
        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, sim.owner):
            raise NRPServicesWrongUserException()

        body = request.get_json(force=True)

        missingParameters = None
        for item in SimulationRobots.RobotPutRequest.required:
            if item not in body:
                missingParameters = item + ' '
        if missingParameters:
            raise NRPServicesClientErrorException('Missing parameter(s): ' + missingParameters)

        try:
            rid = body.get(Parameters.ROBOT_ID)
            rpath = body.get(Parameters.ROBOT_REL_PATH)
            isCustom = False
            if Parameters.IS_CUSTOM in body and body.get(Parameters.IS_CUSTOM) == "True":
                isCustom = True
            res, err = SimulationRobots.__add_new_robot(sim, rid, rpath, isCustom)

        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        if not res:
            return {'res': err}, 404
        return {'res': 'success'}, 200

    @staticmethod
    def __add_new_robot(sim, robot_id, robot_model_rel_path, is_custom):
        """
        Add a new robot
        """
        return sim.cle.add_simulation_robot(robot_id, robot_model_rel_path, is_custom)

    @staticmethod
    def __get_simulation_robots(sim):
        """
        Get simulation robots

        :param sim:
        """
        return sim.cle.get_simulation_robots()
