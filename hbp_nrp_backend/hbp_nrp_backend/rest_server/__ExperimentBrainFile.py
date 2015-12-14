"""
This module contains the REST implementation
for loading and saving experiment brain files
"""


__author__ = 'Bernd Eckstein'

from flask_restful import Resource, fields, request
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import \
    ErrorMessages
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication


# pylint: disable=R0201
# because it seems to be buggy:
# pylint: disable=W0105


class ExperimentBrainFile(Resource):
    """
    The resource to save experiment brain files to the collab
    """

    @swagger.model
    class _Brain(object):
        """
        Set Experiment brain
        Only used for swagger documentation
        """

        resource_fields = {
            'data': fields.String()
        }
        required = ['data']

    @swagger.operation(
        notes='Save a brain model PyNN of an experiment to the collab.',
        parameters=[
            {
                "name": "brain_model",
                "required": True,
                "description": "Brain model in the Python language (using PyNN)",
                "paramType": "body",
                "dataType": _Brain.__name__
            },
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.ERROR_SAVING_FILE_500
            },
            {
                "code": 404,
                "message": ErrorMessages.COLLAB_NOT_FOUND_404
            },
            {
                "code": 400,
                "message": "Neural network python code should be sent in "
                           "the body under the 'data' key"
            },
            {
                "code": 200,
            }
        ]
    )
    def put(self, context_id):
        """
         Save a brain model PyNN of an experiment to the collab.

        :param path context_id: The context UUID of the Collab where the transfer functions
         will be saved
        :<json body json string data: PyNN script of the model
        :status 500: Error saving file
        :status 404: The collab with the given context ID was not found
        :status 400: The request body is malformed
        :status 200: Success. File written.
        """

        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient

        body = request.get_json(force=True)
        if (not 'data' in body):
            raise NRPServicesClientErrorException(
                "Neural network python code should be sent in the body under the 'data' key"
            )

        data = body['data']

        client = NeuroroboticsCollabClient(
            UserAuthentication.get_header_token(request),
            context_id
        )

        client.save_string_to_file_in_collab(
            data,
            client.BRAIN_PYNN_MIMETYPE,
            "recovered_pynn_brain_model.py"
        )

        return 200
