"""
This module contains the REST implementation
for loading and saving experiment state machine files
"""


__author__ = 'Bernd Eckstein'

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import save_file, \
    ErrorMessages, get_control_state_machine_files, get_evaluation_state_machine_files

import os

# pylint: disable=no-self-use


@swagger.model
class StateMachines(object):
    """
    Get and Set Experiment State Machine
    Only used for swagger documentation
    """

    resource_fields = {
        'StateMachine_1': fields.String(),
        'StateMachine_2': fields.String()
    }


@swagger.model
@swagger.nested(control=StateMachines.__name__,
                evaluation=StateMachines.__name__)
class ReturnObject(object):
    """
    Get and Set Experiment State Machine
    Only used for swagger documentation
    """

    resource_fields = {
        'control': fields.Nested(StateMachines.resource_fields),
        'evaluation': fields.Nested(StateMachines.resource_fields)
    }
    required = ['control', 'evaluation']


class ExperimentGetStateMachines(Resource):
    """
    The resource to load experiment state machine files
    """

    @swagger.operation(
        notes='Get the state machine files of a given experiment.',
        responseClass=ReturnObject.__name__,
        parameters=[
            {
                "name": "exp_id",
                "description": "The ID of the experiment state machine files to be retrieved",
                "required": True,
                "paramType": "path",
                "dataType": basestring.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.VARIABLE_ERROR
            },
            {
                "code": 404,
                "message": ErrorMessages.EXPERIMENT_NOT_FOUND_404
            },
            {
                "code": 404,
                "message": "State machine file not found: [filename]"
            },
            {
                "code": 200,
                "message": "Success. The experiment state machine files where retrieved"
            }
        ]
    )
    def get(self, exp_id):
        """
        Gets state machine files of the experiment specified with experiment ID.

        :param exp_id: The experiment ID
        :>json dict control: Contains control state machines as dictionary ('name': 'source_code')
        :>json dict evaluation: Contains evaluation state machines as dictionary \
        ('name': 'source_code')
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: State machine file not found: [filename]
        :status 404: The experiment with the given ID was not found
        :status 200: Success. The experiment state machine files where retrieved
        """

        # Get ID, absolute path of state machine files
        ctrl_file_names = get_control_state_machine_files(exp_id)
        eval_file_names = get_evaluation_state_machine_files(exp_id)

        # Crete return dictionaries
        ctrl_ret = dict()
        eval_ret = dict()

        # So we can iterate over it and don't have to copy-paste the for-loop below: create Arrays
        file_names = [ctrl_file_names, eval_file_names]
        ret = [ctrl_ret, eval_ret]

        for i in 0, 1:
            for sm in file_names[i]:
                filename = file_names[i][sm]
                if not os.path.isfile(filename):
                    raise NRPServicesClientErrorException("State machine file not found: " +
                                                          filename, 404)
                with open(filename, "r") as _file:
                    data = _file.read()
                    ret[i][sm] = data

        return {'control': ctrl_ret, 'evaluation': eval_ret}, 200


class ExperimentPutStateMachine(Resource):
    """
    The resource to save experiment state machine files
    """

    @swagger.operation(
        notes='Sends a state machine file of an experiment to the server',
        parameters=[
            {
                "name": "exp_id",
                "required": True,
                "description": "The ID of the experiment, whose BIBI file will be written.",
                "paramType": "path",
                "dataType": basestring.__name__
            },
            {
                "name": "state_machine_name",
                "required": True,
                "description": "The name of the state machine to be written",
                "paramType": "path",
                "dataType": basestring.__name__
            },
            {
                "name": "source_code",
                "description": "The source code of the state machine",
                "required": True,
                "paramType": "body",
                "dataType": basestring.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.VARIABLE_ERROR
            },
            {
                "code": 404,
                "message": ErrorMessages.EXPERIMENT_NOT_FOUND_404
            },
            {
                "code": 200,
                "message": "Success. File written: '[file_name]'"
            }
        ]
    )
    def put(self, exp_id, state_machine_name):
        """
        Send a state machine of an experiment to the server
        (currently, the file will be written with the ending "_new")

        :param path exp_id: The experiment ID
        :param path state_machine_name: The name of the state machine to be written
        :param body source_code: Source code of the state machine as string.
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 404: The experiment with the given ID was not found
        :status 404: State machine not found: [state_machine_name]
        :status 200: Success. File written.
        """

        state_machine_source = request.data

        ctrl_file_names = get_control_state_machine_files(exp_id)
        eval_file_names = get_evaluation_state_machine_files(exp_id)

        if state_machine_name not in ctrl_file_names and state_machine_name not in eval_file_names:
            raise NRPServicesClientErrorException("State machine not found: "
                                                  "{0}".format(state_machine_name), 404)

        if state_machine_name in ctrl_file_names:
            filename = ctrl_file_names[state_machine_name]
        else:
            filename = eval_file_names[state_machine_name]

        filename = save_file(None, filename, state_machine_source)
        return {'message': "Success. File written: {0}".format(filename)}, 200
