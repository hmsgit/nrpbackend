"""
This module contains the REST implementation for the simulation control
"""

__author__ = 'GeorgHinkel'

from flask import request
from flask_restful import Resource, abort, marshal_with, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server.__UserAuthentication import \
    UserAuthentication

from std_msgs.msg import ColorRGBA
from gazebo_msgs.srv import SetVisualProperties, SetLightProperties, GetLightProperties
import rospy

# pylint: disable=R0201


def _get_simulation_or_abort(sim_id):
    """
    Gets the simulation with the given simulation id or aborts the simulation otherwise
    :param sim_id: The simulation id
    """
    if sim_id < 0 or sim_id >= len(simulations):
        abort(404)
    return simulations[sim_id]


class SimulationControl(Resource):
    """
    The resource to control the simulation
    """

    @swagger.operation(
        notes='Gets the simulation with given sim_id',
        responseClass=Simulation.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "required": True,
                "allowMultiple": False,
                "paramType": "path",
                "dataType": int.__name__
            }
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 200,
                "message": "Success. The simulation with the given ID is retrieved"
            }
        ]
    )
    @marshal_with(Simulation.resource_fields)
    def get(self, sim_id):
        """
        Gets the simulation with the specified simulation id
        """
        return _get_simulation_or_abort(sim_id), 200


@swagger.model
class _RGBADescription(object):
    """Describe a RGBA color"""
    resource_fields = {
        'r': fields.Float,
        'g': fields.Float,
        'b': fields.Float,
        'a': fields.Float,
    }


@swagger.model
@swagger.nested(diffuse=_RGBADescription.__name__, )
class _LightDescription(object):
    """Describe a light"""
    resource_fields = {
        'name': fields.String,
        'diffuse': fields.Nested(_RGBADescription.resource_fields),
        'attenuation_constant': fields.Float,
        'attenuation_linear': fields.Float,
        'attenuation_quadratic': fields.Float,
    }


class LightControl(Resource):
    """
    The resource to change the light intensity
    """

    @swagger.operation(
        notes='Controls the light of the given light source',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation on which light should be changed",
                "paramType": "path",
                "required": True,
                "dataType": int.__name__
            },
            {
                "name": "light",
                "description": "Description of the light to change",
                "paramType": "body",
                "required": True,
                "dataType": _LightDescription.__name__
            }
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 400,
                "message": "The parameters are invalid"
            },
            {
                "code": 401,
                "message": "Only allowed by simulation owner"
            },
            {
                "code": 200,
                "message": "Success. "
            }
            # pylint: disable=R0911
        ]
    )
    def put(self, sim_id):
        """
        Raises a hardware event
        :param sim_id: The simulation id
        """
        simulation = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            return "You need to be the simulation owner to trigger interactions", 401

        body = request.get_json(force=True)
        if 'name' not in body:
            return "No name given", 400

        in_name = body['name']
        in_diffuse = body.get('diffuse')
        in_attenuation_constant = body.get('attenuation_constant')
        in_attenuation_linear = body.get('attenuation_linear')
        in_attenuation_quadratic = body.get('attenuation_quadratic')

        diffuse = None

        if in_diffuse is not None:
            diffuse = ColorRGBA(in_diffuse['r'], in_diffuse['g'],
                                in_diffuse['b'], in_diffuse['a'])

        if in_diffuse is None or in_attenuation_constant is None \
                or in_attenuation_linear is None or in_attenuation_quadratic is None:

            try:
                rospy.wait_for_service('/gazebo/get_light_properties', 1)
            except rospy.ROSException as exc:
                return "ROS service not available: " + str(exc), 400

            get_light_properties = rospy.ServiceProxy('/gazebo/get_light_properties',
                                                      GetLightProperties)

            try:
                light_properties = get_light_properties(light_name=in_name)

                if in_attenuation_constant is None:
                    in_attenuation_constant = light_properties.attenuation_constant
                if in_attenuation_linear is None:
                    in_attenuation_linear = light_properties.attenuation_linear
                if in_attenuation_quadratic is None:
                    in_attenuation_quadratic = light_properties.attenuation_quadratic
                if in_diffuse is None:
                    diffuse = light_properties.diffuse

            except rospy.ServiceException as exc:
                return "Service did not process request:" + str(exc), 400

        try:
            rospy.wait_for_service('/gazebo/set_light_properties', 1)
        except rospy.ROSException as exc:
            return "ROS service not available: " + str(exc), 400

        set_light_properties = rospy.ServiceProxy('/gazebo/set_light_properties',
                                                  SetLightProperties)

        try:
            set_light_properties(light_name=in_name,
                                 diffuse=diffuse,
                                 attenuation_constant=in_attenuation_constant,
                                 attenuation_linear=in_attenuation_linear,
                                 attenuation_quadratic=in_attenuation_quadratic)
        except rospy.ServiceException as exc:
            return "Service did not process request: " + str(exc), 400

        return "Changed light intensity", 200


class CustomEventControl(Resource):
    """
    The resource to raise custom events
    """

    @swagger.model
    class _CustomEvent(object):
        """Describe an event"""
        resource_fields = {
            'name': fields.String,
        }

    def __set_light(self, vr_name, color):
        """
        Sets the light of a particular visual
        :param vr_name: The visual name
        :param color: The color
        :return: The response
        """
        try:
            rospy.wait_for_service('/gazebo/set_visual_properties', 1)
        except rospy.ROSException as exc:
            return "ROS service not available: " + str(exc), 400
        set_visual_properties = rospy.ServiceProxy('/gazebo/set_visual_properties',
                                                   SetVisualProperties)
        try:
            set_visual_properties(model_name=vr_name, link_name='body',
                                  visual_name='screen_glass',
                                  property_name='material:script:name',
                                  property_value=color)
        except rospy.ServiceException as exc:
            return "Service did not process request: " + str(exc), 400
        return "Changed color", 200

    @swagger.operation(
        notes='Currently, only RightScreenToRed is implemented',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation on which to raise the event",
                "paramType": "path",
                "required": True,
                "dataType": int.__name__
            },
            {
                "name": "event",
                "description": "Custom event to raise",
                "paramType": "body",
                "required": True,
                "dataType": _CustomEvent.__name__
            }
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation or interaction was not found"
            },
            {
                "code": 400,
                "message": "The parameters are invalid"
            },
            {
                "code": 401,
                "message": "Only allowed by simulation owner"
            },
            {
                "code": 200,
                "message": "Success. "
            }
            # pylint: disable=R0911
        ]
    )
    def put(self, sim_id):
        """
        Raises a hardware event
        :param sim_id: The simulation id
        """
        simulation = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            return "You need to be the simulation owner to trigger interactions", 401

        body = request.get_json(force=True)
        if 'name' not in body:
            return "No name given", 400
        name = body['name']
        if name == 'RightScreenToRed':
            return self.__set_light('right_vr_screen', 'Gazebo/Red')
        if name == "RightScreenToBlue":
            return self.__set_light('right_vr_screen', 'Gazebo/Blue')
        if name == "LeftScreenToRed":
            return self.__set_light('left_vr_screen', 'Gazebo/Red')
        if name == "LeftScreenToBlue":
            return self.__set_light('left_vr_screen', 'Gazebo/Blue')
        return "Interaction not found", 404
