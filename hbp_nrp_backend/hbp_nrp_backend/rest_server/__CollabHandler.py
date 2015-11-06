"""
This module contains the REST implementation
that deals with the collaboratory platform
"""

__author__ = 'jkaiser'

from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger
from hbp_nrp_backend.rest_server import db
# Import data base models
from hbp_nrp_backend.rest_server.__CollabContext import CollabContext
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException

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
        # pylint: disable=no-member
        collab_context = CollabContext.query.get_or_404(context_id)
        return {
            'contextID': context_id,
            'experimentID': str(collab_context.experiment_id)
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

        body = request.get_json(force=True)
        if 'experimentID' not in body:
            raise NRPServicesClientErrorException("No experimentID given")
        experiment_id = body['experimentID']
        # pylint: disable=no-member
        db.session.add(CollabContext(context_id, experiment_id))
        db.session.commit()

        return {'experimentID': experiment_id,
                'contextID': context_id}, 200
