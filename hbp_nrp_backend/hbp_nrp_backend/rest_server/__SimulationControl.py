"""
This module contains the REST implementation for the simulation control
"""

__author__ = 'GeorgHinkel'

from flask import request
from flask_restful import Resource, abort, marshal_with
from flask_restful_swagger import swagger

from hbp_nrp_backend.simulation_control import simulations, Simulation, \
    InvalidStateTransitionException

# from gazebo_msgs.srv import SetVisualProperties
# import rospy

# __set_visual_properties = rospy.ServiceProxy('/gazebo/set_visual_properties', SetVisualProperties)
# __set_light_properties = rospy.ServiceProxy('/gazebo/set_light_properties', None)

# pylint: disable=R0201
# pylint: disable=W0612


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
        notes='Gets the state of the given simulation',
        responseClass=Simulation.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "paramType": "requestURI",
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
        :param sim_id: The simulation id
        """
        return _get_simulation_or_abort(sim_id)

    @swagger.operation(
        notes='Sets the state of the given simulation',
        responseClass=Simulation.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "paramType": "requestURI",
                "dataType": int.__name__
            },
            {
                "name": "state",
                "description": "The new state of the simulation in question",
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
                "message": "The state transition is invalid"
            },
            {
                "code": 200,
                "message": "Success. The simulation with the given ID is retrieved"
            }
        ]
    )
    @marshal_with(Simulation.resource_fields)
    def put(self, sim_id):
        """
        Sets the simulation with the given name into a new state
        :param sim_id: The simulation id
        """
        simulation = _get_simulation_or_abort(sim_id)
        body = request.get_json(force=True)
        try:
            simulation.state = body['state']
        except InvalidStateTransitionException:
            return "Invalid state transition", 400
        return simulation


class LightControl(Resource):
    """
    The resource to change the light intensity
    """
    @swagger.operation(
        notes='Controls the light of the given light source',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "paramType": "requestURI",
                "dataType": int.__name__
            },
            {
                "name": "name",
                "description": "The name of the light source",
                "paramType": "body",
                "dataType": str.__name__
            },
            {
                "name": "attenuation_constant",
                "description": "The constant attenuation of the light",
                "paramType": "body",
                "dataType": float.__name__
            },
            {
                "name": "attenuation_linear",
                "description": "The linear attenuation of the light",
                "paramType": "body",
                "dataType": float.__name__
            },
            {
                "name": "attenuation_quadratic",
                "description": "The quadratic attenuation of the light",
                "paramType": "body",
                "dataType": float.__name__
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
                "code": 200,
                "message": "Success. "
            }
        ]
    )
    def put(self, sim_id):
        """
        Raises a hardware event
        :param sim_id: The simulation id
        """
        simulation = _get_simulation_or_abort(sim_id)
        body = request.get_json(force=True)
        if not 'name' in body:
            return "No name given", 400
        name = body['name']
        diffuse = body.get('diffuse')
        attenuation_constant = body.get('attenuation_constant', 0.0)
        attenuation_linear = body.get('attenuation_linear', 0.0)
        attenuation_quadratic = body.get('attenuation_quadratic', 0.0)


class CustomEventControl(Resource):
    """
    The resource to raise custom events
    """
    @swagger.operation(
        notes='Currently, only RightScreenToRed is implemented',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "paramType": "requestURI",
                "dataType": int.__name__
            },
            {
                "name": "name",
                "description": "The name of the custom event that should be raised",
                "paramType": "body",
                "dataType": str.__name__
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
                "code": 200,
                "message": "Success. "
            }
        ]
    )
    def put(self, sim_id):
        """
        Raises a hardware event
        :param sim_id: The simulation id
        """
        simulation = _get_simulation_or_abort(sim_id)
        body = request.get_json(force=True)
        if not 'name' in body:
            return "No name given", 400
        name = body['name']
        if name == 'RightScreenToRed':
            # __set_visual_properties(model_name='right_vr_screen', link_name='body',
            #  visual_name='screen_glass',
            #                         property_name='material:script:name',
            #                         property_value='Gazebo/Red')
            return "Changed color", 200
        return "Interaction not found", 404
