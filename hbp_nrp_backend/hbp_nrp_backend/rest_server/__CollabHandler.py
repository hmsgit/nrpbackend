"""
This module contains the REST implementation
that deals with the collaboratory platform
"""

__author__ = 'jkaiser'

import logging

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger
from hbp_nrp_backend.rest_server import db

# Import data base models
from hbp_nrp_backend.rest_server.__CollabContext import CollabContext
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_backend.rest_server.__ExperimentService import get_experiment_conf
from hbp_nrp_backend.rest_server.__CollabContext import get_or_raise as get_or_raise_collab_context

logger = logging.getLogger(__name__)

# pylint: disable=no-self-use


class CollabHandler(Resource):
    """
    The resource managing Collab context UUIDs and associated experiment IDs
    """
    def __init__(self):
        Resource.__init__(self)

    @swagger.model
    class _CollabHandler(object):
        """
        Experiment configuration that links a context UUID with an experiment ID.
        When the user selects an experiment to clone from the collab edit page,
        we store the ID of that experiment in a database
        Only used for swagger documentation
        """

        resource_fields = {
            'experimentID': fields.String(),
            'contextID': fields.String()
        }
        required = ['experimentID', 'contextID']

    @swagger.operation(
        notes='Retrieves an experiment ID based on a Collab context UUID',
        responseClass=_CollabHandler.__name__,
        parameters=[
            {
                "name": "context_id",
                "description": "The UUID of the Collab context paired with \
                the requested experiment ID",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            }
        ],
        responseMessages=[
            {
                "code": 404,
                "message": "The experiment ID was not found"
            },
            {
                "code": 200,
                "message": "Success. The experiment ID was retrieved"
            }
        ]
    )
    def get(self, context_id):
        """
        Gets the experiment ID

        :param context_id: The Collab context UUID
        :status 404: The experiment ID associated with the given context UUID was not found
        :status 200: The experiment ID was successfully retrieved
        """
        # pylint does not recognise members created by SQLAlchemy
        collab_context = get_or_raise_collab_context(context_id)

        experiment_id = ""
        experiment_folder_uuid = ""
        if collab_context is not None:
            experiment_id = str(collab_context.experiment_id)
            experiment_folder_uuid = str(collab_context.experiment_folder_uuid)
        return {
            'contextID': context_id,
            'experimentID': experiment_id,
            'experimentFolderUUID': experiment_folder_uuid
        }, 200

    @swagger.operation(
        notes='Saves a key-value pair made of a Collab context UUID and an experiment ID',
        responseClass=_CollabHandler.__name__,
        parameters=[
            {
                "name": "context_id",
                "description": "The UUID of the Collab context to be paired with \
                the ID of the selected experiment",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            },
            {
                "name": "experimentID",
                "description": "The ID of the selected experiment",
                "required": True,
                "paramType": "body",
                "dataType": str.__name__
            }
        ],
        responseMessages=[
            {
                "code": 400,
                "message": "No experimentID given"
            },
            {
                "code": 200,
                "message": "Success. The context UUID and its \
                associated experiment ID have been saved."
            }
        ]
    )
    def put(self, context_id):
        """
        Saves a key-value pair associating a Collab context UUID and \
        an experiment ID

        :param context_id: The Collab context UUID
        :status 400: No experimentID given.
        :status 200: The Collab context and its associated experiment ID were successfully retrieved
        """

        # pylint: disable=no-member
        collab_context = get_or_raise_collab_context(context_id)
        if collab_context is not None:
            # In the future, we may allow people to recreate experiment using the same
            # context (this should erase the given directory and clean up the storage).
            return "Forbidden to override a given experiment template.", 409

        body = request.get_json(force=True)
        if 'experimentID' not in body:
            raise NRPServicesClientErrorException("No experimentID given")
        experiment_id = body['experimentID']

        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient
        client = NeuroroboticsCollabClient(UserAuthentication.get_header_token(request),
                                           context_id)
        exd_configuration = get_experiment_conf(experiment_id)
        experiment_folder_uuid = client.clone_experiment_template_to_collab(
            client.generate_unique_folder_name(client.get_context_app_name()),
                                                   exd_configuration)

        db.session.add(CollabContext(context_id, experiment_id, experiment_folder_uuid))
        db.session.commit()

        client.add_app_to_nav_menu()

        return {'experimentID': experiment_id,
                'contextID': context_id,
                'experimentFolderUUID': experiment_folder_uuid}, 200
