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
from hbp_nrp_backend.rest_server import NRPServicesGeneralException, \
    NRPServicesStateMachineException, NRPServicesWrongUserException
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_backend.simulation_control.__Simulation import Simulation
from sys import exc_info

# pylint: disable=no-self-use

logger = logging.getLogger(__name__)


def reset_simulation_or_raise(simulation):
    """
    Reset the simulation and raise an exception in case of failure.
    This is needed before an update of a state machine code
    :param simulation: The simulation to be reset
    """
    state = simulation.state
    if state == 'paused' or state == 'started':
        state = simulation.state = 'paused'

    if state != 'paused':
        raise NRPServicesGeneralException(
            "Simulation in state {0}. Can't update the state machine".format(state),
            "Server error"
        )


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


class SimulationStateMachines(Resource):
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

        for sm in simulation.state_machines:
            state_machines[sm.sm_id] = simulation.get_state_machine_code(sm.sm_id)

        return dict(data=state_machines), 200


class SimulationStateMachine(Resource):
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

        :status 400: The source code doesn't describe a valid SMACH state machine
        :status 400: The source code is invalid: [ERROR-MESSAGE]
        :status 401: Operation only allowed by simulation owner
        :status 404: The simulation with the given ID was not found
        :status 404: The state machine with the given name was not found
        :status 500: Simulation in state [STATE]. Can't update state machine.
        :status 200: Success. The code was successfully patched
        """
        simulation = _get_simulation_or_abort(sim_id)
        assert simulation, Simulation
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        state_machine_source = request.data
        reset_simulation_or_raise(simulation)
        try:
            simulation.set_state_machine_code(
                state_machine_name,
                state_machine_source
            )
            return "Success. The code was successfully patched.", 200
        except (AttributeError, NameError) as e:
            info = exc_info()
            raise NRPServicesStateMachineException(e.message, 400), None, info[2]

        except SyntaxError as e:
            info = exc_info()
            args_txt = ""
            for text in e.args:
                args_txt += " {0}".format(text)
            raise NRPServicesStateMachineException(
                "The source code is invalid: "
                "SyntaxError in line {0}{1}.".format(e.lineno, args_txt),
                400
            ), None, info[2]

        except Exception as e:
            info = exc_info()
            raise NRPServicesGeneralException(
                "Update of state machine code failed. "
                "{0}: {1}".format(
                    e.__class__.__name__,
                    e.message
                ), "State machine error"
            ), None, info[2]

    @swagger.operation(
        notes='Delete a state machine.',
        responseClass=int.__name__,
        parameters=[
            {
                "name": "sim_id",
                "required": True,
                "description": "The ID of the simulation whose state machine will be deleted",
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "state_machine_name",
                "description": "The name of the state machine to delete",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 200,
                "message": "Success. The delete operation was successfully called. This "
                           "does not imply that the state machine function was correctly "
                           "deleted though."
            }
        ]
    )
    def delete(self, sim_id, state_machine_name):
        """
        Delete a state machine

        :param sim_id: The simulation id
        :param state_machine_name: The name of the transfer function to delete
        :status 401: Insufficient permissions to apply changes
        :status 404: The simulation with the given ID was not found
        :status 404: The state machine with the given name was not found
        :status 500: Server error with specific message.
        :status 200: The delete operation was successfully called
        """
        simulation = _get_simulation_or_abort(sim_id)
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        failure_message = "State machine destruction failed: "
        response_message = None
        try:
            ok, response_message = simulation.delete_state_machine(state_machine_name)
            if ok:
                return "Success. The state machine was successfully deleted.", 200
        except Exception as e:
            info = exc_info()
            raise NRPServicesGeneralException(
                failure_message +
                " {0}: {1}".format(e.__class__.__name__, e.message),
                "State machine error",
            ), None, info[2]

        raise NRPServicesStateMachineException(
            failure_message + "\n" +
            response_message,
            404
        )
