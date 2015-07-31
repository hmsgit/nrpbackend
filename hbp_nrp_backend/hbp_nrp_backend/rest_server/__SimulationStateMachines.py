"""
This module contains the REST implementation
for setting and getting the experiment control
state maching in a running simulation
"""

__author__ = 'Bernd Eckstein'

import logging
from flask import request
from flask_restful_swagger import swagger
from flask_restful import Resource, fields
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, NRPServicesGeneralException
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_backend.simulation_control.__Simulation import Simulation

# pylint: disable=no-self-use

logger = logging.getLogger(__name__)


@swagger.model
class StateMachineDictionary(object):
    """
    Swagger documentation object
    StateMachineDict ... tried to make it look like a dictionary for the swagger doc
    """
    resource_fields = {
        'sm_id_1': str.__name__,
        'sm_id_2': str.__name__,
        'sm_id_n': str.__name__
    }


@swagger.model
@swagger.nested(data=StateMachineDictionary.__name__)
class StateMachineData(object):
    """
    Swagger documentation object
    Main Data Attribute for parsing convenience on the frontend side.
    """
    resource_fields = {
        'data': fields.Nested(StateMachineDictionary.resource_fields)
    }
    required = ['data']


class SimulationGetStateMachine(Resource):
    """
    REST service for getting the code of all state machines
    """

    def __init__(self):
        Resource.__init__(self)

    @swagger.operation(
        notes='Get code of all state machines',
        responseClass=StateMachineData.__name__,
        parameters=[
            {
                "name": "sim_id",
                "required": True,
                "description": "The ID of the simulation whose state machine will be modified",
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
                "message": "Success. The state machines where returned"
            }
        ]
    )
    def get(self, sim_id):
        """
        Get code of all state machines.
        This works in all simulation states except 'created', where it will deliver an empty
        dictionary.

        :param sim_id: The simulation id
        :>json dict data: Dictionary containing all state machines ('name': 'source')
        :status 404: The simulation with the given ID was not found
        :status 200: Success. The state machines where returned
        """

        simulation = _get_simulation_or_abort(sim_id)

        state_machines = dict()

        for sm in simulation.state_machines.keys():
            state_machines[sm] = simulation.get_state_machine_code(sm)

        return dict(data=state_machines), 200


class SimulationPutStateMachine(Resource):
    """
    REST service for patching control state machine source code.
    """

    @swagger.operation(
        notes='Applies user changes to state machine code.',
        responseClass=int.__name__,
        parameters=[
            {
                "name": "sim_id",
                "required": True,
                "description": "The ID of the simulation whose state machine will be modified",
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "state_machine_name",
                "description": "The name of the state machine",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
            {
                "name": "source_code",
                "description": "The state machine source code to patch",
                "required": True,
                "paramType": "body",
                "dataType": str.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": "Simulation in state [STATE]. Can't update state machine."
            },
            {
                "code": 404,
                "message": "The simulation with the given ID was not found"
            },
            {
                "code": 404,
                "message": "State machine not found."
            },
            {
                "code": 400,
                "message": "The passed source code does not describe a valid SMACH statemachine"
            },
            {
                "code": 400,
                "message": "The source code is invalid: [ERROR-MESSAGE]"
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
    def put(self, sim_id, state_machine_name):
        """
        Applies user changes to state machine code.
        When the simulation state is 'paused' or 'started', the simulation is reset
        and the new state is 'initialized'.
        When the Simulation state is 'stopped', 'created' or 'failed', nothing happens.

        :param path sim_id: The simulation id
        :param path state_machine_name: The state machine name
        :param body source_code: The source code of the state machine

        :status 400: The passed source code does not describe a valid SMACH statemachine
        :status 400: The source code is invalid: [ERROR-MESSAGE]
        :status 401: Operation only allowed by simulation owner
        :status 404: The simulation with the given ID was not found
        :status 404: State machine not found.
        :status 500: Simulation in state [STATE]. Can't update state machine.
        :status 200: Success. The code was successfully patched
        """
        simulation = _get_simulation_or_abort(sim_id)
        assert simulation, Simulation
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesClientErrorException(
                "You need to be the simulation owner to apply your changes", 401)

        state_machine_source = request.data

        # Set state to initialized when paused or started. (This is needed to set a state machine)
        if simulation.state == 'paused' or simulation.state == 'started':
            simulation.state = 'initialized'

        if simulation.state != 'initialized':
            raise NRPServicesGeneralException(
                "Simulation in state {0}. Can't update the state machine".format(simulation.state),
                "Server error")

        try:
            ok, message = simulation.set_state_machine_code(state_machine_name,
                                                            state_machine_source)
            if ok:
                return "Success. The code was successfully patched.", 200
            else:
                return "{0}".format(message), 404
        except AttributeError as e:
            raise NRPServicesClientErrorException(e.message, 400)
        except SyntaxError as e:
            args_txt = ""
            for text in e.args:
                args_txt += " {0}".format(text)
            raise NRPServicesClientErrorException(
                "The source code is invalid: "
                "SyntaxError in line {0}{1}.".format(e.lineno, args_txt), 400)
        except Exception as e:
            raise NRPServicesClientErrorException(
                "The source code is invalid. "
                "{0}: {1}".format(e.__class__.__name__, e.message), 400)
