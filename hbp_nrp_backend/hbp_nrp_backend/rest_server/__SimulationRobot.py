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

from hbp_nrp_backend import NRPServicesGeneralException, \
    NRPServicesWrongUserException, NRPServicesClientErrorException
from hbp_nrp_backend.rest_server import ErrorMessages, ParamNames

from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.__UserAuthentication import UserAuthentication

logger = logging.getLogger(__name__)


class SimulationRobots(Resource): # pragma: no cover
    """
    This resource handles the robot models in a running simulation
    """
    @swagger.operation(
        notes='Gets the list of robots in the running simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose robot list shall be retrieved",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
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
    def get(self, sim_id):
        """
        Gets the list of robots in the running simulation.
        """
        # pylint: disable=no-self-use
        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.can_view(sim):
            raise NRPServicesWrongUserException()

        try:
            robots = SimulationRobots.__get_simulation_robots(sim)
        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        result = {
            'robots': [{
                           ParamNames.ROBOT_ID: robot.robot_id,
                           ParamNames.ROBOT_ABS_PATH: robot.robot_model,
                           ParamNames.IS_CUSTOM: robot.is_custom,
                           ParamNames.ROBOT_POSE: SimulationRobots.__jsonisePose(robot.pose)
                       } for robot in robots
                       ]
        }
        return result, 200

    @staticmethod
    def __get_simulation_robots(sim):
        """
        Reset populations

        :param sim:
        """
        return sim.cle.get_simulation_robots()

    @staticmethod
    def __jsonisePose(pose):
        """
        Convert a cle_ros_msgs.msg.Pose to returnable json
        :param pose: a cle_ros_msgs.msg.Pose
        :return:
        """
        return {
            'x': pose.x,
            'y': pose.y,
            'z': pose.z,
            'roll': pose.roll,
            'pitch': pose.pitch,
            'yaw': pose.yaw
        } if pose else {}


class SimulationRobot(Resource): # pragma: no cover
    """
    This resource handles the robot models in a running simulation
    """
    @swagger.model
    class RobotRequest(object):
        """
        Represents a request for the API implemented by SimulationRobot
        """
        resource_fields = {
            ParamNames.ROBOT_MODEL: fields.String,
            ParamNames.IS_CUSTOM: fields.Boolean,
            ParamNames.ROBOT_POSE: fields.String
        }

    @swagger.model
    class RobotPostRequest(RobotRequest):
        """
        Lists required fields for a POST new pose request
        """
        required = [ParamNames.ROBOT_MODEL]

    @swagger.model
    class RobotPutRequest(RobotRequest):
        """
        Lists required fields for a PUT request
        """
        required = [ParamNames.ROBOT_POSE]

    @swagger.operation(
        notes='Adds a new robot to the simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation in which robot would be added",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "robot_id",
                "description": "The string ID of the robot to be deleted",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
            {
                "name": "body",
                "paramType": "body",
                "dataType": RobotPostRequest.__name__,
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
    def post(self, sim_id, robot_id):
        """
        Add a new robot to the simulation.

        :param sim_id: The simulation ID.
        :param robot_id: The robot string ID.

        :status 500: {0}
        :status 404: {1}
        :status 401: {2}
        :status 400: Invalid request, the JSON parameters are incorrect
        :status 200: The requested robot was created successfully
        """
        # pylint: disable=no-self-use
        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.can_modify(sim):
            raise NRPServicesWrongUserException()

        body = request.get_json(force=True)

        missing_parameters = [item for item in SimulationRobot.RobotPostRequest.required
                                if item not in body]
        if missing_parameters:
            raise NRPServicesClientErrorException('Missing parameter(s): ' +
                                                  ' '.join(missing_parameters))

        try:
            res, err = SimulationRobot.__add_new_robot(
                sim=sim,
                robot_id=robot_id,
                robot_model=body.get(ParamNames.ROBOT_MODEL),
                is_custom=(body.get(ParamNames.IS_CUSTOM, None) == 'True'),
                initial_pose=body.get(ParamNames.ROBOT_POSE, None)
            )
        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        if not res:
            return {'res': err}, 404
        return {'res': 'success'}, 200

    @swagger.operation(
        notes='Deletes robot from the simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose robot would be deleted",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "robot_id",
                "description": "The string ID of the robot to be deleted",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            }
        ],
        responseMessages=[
            {"code": 500, "message": ErrorMessages.SERVER_ERROR_500},
            {"code": 404, "message": ErrorMessages.SIMULATION_NOT_FOUND_404},
            {"code": 401, "message": ErrorMessages.SIMULATION_PERMISSION_401},
            {"code": 400, "message": "Invalid request, the JSON parameters are incorrect."},
            {"code": 200, "message": "Success. Robot deleted."},
        ]
    )
    def delete(self, sim_id, robot_id):
        """
        Delete a robot from the simulation.

        :param sim_id: The simulation ID.
        :param robot_id: The robot ID.

        :status 500: {0}
        :status 404: {1}
        :status 401: {2}
        :status 400: Invalid request, the JSON parameters are incorrect
        :status 200: The requested robot was deleted successfully
        """
        # pylint: disable=no-self-use
        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.can_modify(sim):
            raise NRPServicesWrongUserException()
        try:
            res, err = SimulationRobot.__delete_robot(sim, robot_id)

        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)
        if not res:
            return {'res': err}, 404
        return {'res': 'success'}, 200

    @swagger.operation(
        notes='Updates initial pose of a robot in the simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation in which robot would be updated",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "robot_id",
                "description": "The ID of the robot that would be updated",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
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
            {"code": 200, "message": "Success. Initial pose successfully updated"},
        ]
    )
    def put(self, sim_id, robot_id):
        """
        Update initial pose of a robot

        :param sim_id: The simulation ID.
        :param robot_id: The robot string ID.

        :status 500: {0}
        :status 404: {1}
        :status 401: {2}
        :status 400: Invalid request, the JSON parameters are incorrect
        :status 200: The requested robot was created successfully
        """
        # pylint: disable=no-self-use
        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.can_modify(sim):
            raise NRPServicesWrongUserException()

        body = request.get_json(force=True)

        missing_parameters = [item for item in SimulationRobot.RobotPutRequest.required
                              if item not in body]
        if missing_parameters:
            raise NRPServicesClientErrorException('Missing parameter(s): ' +
                                                  ' '.join(missing_parameters))

        try:
            rpose = body.get(ParamNames.ROBOT_POSE)
            res, err = SimulationRobot.__set_robot_initial_pose(sim, robot_id, rpose)

        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        if not res:
            return {'res': err}, 404
        return {'res': 'success'}, 200

    @staticmethod
    def __add_new_robot(sim, robot_id, robot_model, is_custom, initial_pose):
        """
        Add a new robot
        """
        return sim.cle.add_simulation_robot(robot_id, robot_model, is_custom, initial_pose)

    @staticmethod
    def __delete_robot(sim, robot_id):
        """
        Delete a robot from simulation
        """
        return sim.cle.delete_simulation_robot(robot_id)

    @staticmethod
    def __set_robot_initial_pose(sim, robot_id, pose):
        """
        Update initial robot pose of a robot
        """
        return sim.cle.set_simulation_robot_initial_pose(robot_id, pose)
