"""
This module contains the REST implementation
that deals with the collaboratory platform
"""

__author__ = 'jkaiser'

from flask_restful import Resource, fields
from flask_restful_swagger import swagger

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
        Experiment configuration that links context UUID with experiment ID.
        When the user selects an experiment to clone from the collab edit page,
        we store the id of that experiment in that database
        Only used for swagger documentation
        """

        resource_fields = {
            'experimentId': fields.String(),
            'contextId': fields.String()
        }
        required = ['experimentId', 'contextId']

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
        return {'experimentId': 1,
                'contextId': context_id}

    @swagger.operation(
        notes='Save a key-value pair made of Collab context UUID and a experiment ID',
        responseClass=_CollabHandler.__name__,
        parameters=[
            {
                "name": "context_id",
                "description": "The UUID of the Collab context to be paired with \
                the ID of the selected experiment",
                "required": True,
                "paramType": "path",
                "dataType": str.__name__
            }
        ],
        responseMessages=[
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
        :status 404: The Collab context data base was not found
        :status 200: The Collab context and its associated experiment ID were successfully retrieved
        """
        return {'experimentId': 1,
                'contextId': context_id}
