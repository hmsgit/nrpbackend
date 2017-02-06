"""
This module contains the REST implementation
for loading and saving experiment state machine files
"""


__author__ = 'Bernd Eckstein'
import os
import tempfile
import shutil
import logging
from threading import Thread
from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import save_file, \
    ErrorMessages, get_control_state_machine_files, get_evaluation_state_machine_files

from hbp_nrp_commons.generated import exp_conf_api_gen
logger = logging.getLogger(__name__)

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
                "message": ErrorMessages.MODEXP_VARIABLE_ERROR
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
                    raise NRPServicesClientErrorException(
                        "State machine file not found: " + filename,
                        error_code=404
                    )
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
                "message": ErrorMessages.MODEXP_VARIABLE_ERROR
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
                                                  "{0}".format(state_machine_name), error_code=404)

        if state_machine_name in ctrl_file_names:
            filename = ctrl_file_names[state_machine_name]
        else:
            filename = eval_file_names[state_machine_name]

        filename = save_file(None, filename, state_machine_source)
        return {'message': "Success. File written: {0}".format(filename)}, 200


class ExperimentCollabStateMachine(Resource):
    """
    The resource to save experiment state machine files
    """
    @swagger.operation(
        notes='Saves state machines of an experiment to the collab',
        parameters=[
            {
                "name": "context_id",
                "required": True,
                "description": "The context_id of the collab.",
                "paramType": "path",
                "dataType": basestring.__name__
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.ERROR_SAVING_FILE_500
            },
            {
                "code": 400,
                "message": "State machine code should be sent in"
                           "the body under the 'state_machines' key"
            },
            {
                "code": 200
            }
        ]
    )
    def put(self, context_id):
        """
        Save state machines to collab

        :param path context_id: The context id of the collab
        :param body source_code: Source code of the state machine as string.
        :status 500: The experiment xml either could not be found or read
        :status 200: Success. File written.
        """
        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient

        body = request.get_json(force=True)
        if (not 'state_machines' in body):
            raise NRPServicesClientErrorException(
                "State machine code should be sent in "
                "the body under the 'state_machines' key"
            )

        client = NeuroroboticsCollabClient(
            UserAuthentication.get_header_token(request),
            context_id
        )
        exp, exp_xml_file_path, exp_remote_path = client.clone_exp_file_from_collab_context()
        threads = []
        for sm_name in body['state_machines']:
            sm_node = exp_conf_api_gen.SMACHStateMachine()
            sm_node.id = os.path.splitext(sm_name)[0]
            sm_node.src = sm_name if sm_name.endswith(".py") else sm_name + ".py"
            exp_control = exp_conf_api_gen.ExperimentControl()
            exp_control.stateMachine.append(sm_node)
            exp.experimentControl = exp_control
            t = Thread(target=client.replace_file_content_in_collab,
                       args=(body['state_machines'][sm_name],
                             client.STATE_MACHINE_PY_MIMETYPE,
                             sm_node.src))
            t.start()
            threads.append(t)

        if tempfile.gettempdir() in exp_xml_file_path:
            logger.debug(
                "removing the temporary experiment xml file %s",
                exp_xml_file_path
            )
            shutil.rmtree(os.path.dirname(exp_xml_file_path))
        t = Thread(target=client.replace_file_content_in_collab,
                   args=(exp.toxml("utf-8"),
                         client.EXPERIMENT_CONFIGURATION_MIMETYPE,
                         exp_remote_path))
        t.start()
        threads.append(t)
        for thread in threads:
            thread.join()
        return {"message": "Success. Files written to collab"}, 200
