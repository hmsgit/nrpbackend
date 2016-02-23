"""
This module contains the REST implementation
for loading and saving experiment brain files
"""


__author__ = 'Bernd Eckstein'

import os
import tempfile
import shutil
import logging

from flask_restful import Resource, fields, request
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, \
    NRPServicesGeneralException
from hbp_nrp_backend.rest_server.__ExperimentService import \
    ErrorMessages
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_commons.generated import bibi_api_gen

logger = logging.getLogger(__name__)

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
            'data': fields.String(),
            'additional_populations': fields.Arbitrary()
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
        :<json body json string brain_populations: neuron populations
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
        if 'data' not in body:
            raise NRPServicesClientErrorException(
                "Neural network python code should be sent in the body under the 'data' key"
            )

        data = body['data']
        brain_populations = body.get('additional_populations')

        client = NeuroroboticsCollabClient(
            UserAuthentication.get_header_token(request),
            context_id
        )

        client.replace_file_content_in_collab(
            data,
            client.BRAIN_PYNN_MIMETYPE,
            "recovered_pynn_brain_model.py"
        )

        bibi_file_path = client.clone_file_from_collab_context(client.BIBI_CONFIGURATION_MIMETYPE)
        if bibi_file_path is None:
            raise NRPServicesGeneralException(
                "BIBI configuration file not found in the Collab storage",
                "BIBI not found"
            )

        with open(bibi_file_path) as bibi_xml:
            bibi = bibi_api_gen.CreateFromDocument(bibi_xml.read())
            if not isinstance(bibi, bibi_api_gen.BIBIConfiguration):
                raise NRPServicesGeneralException(
                    "BIBI configuration file content is not valid.",
                    "BIBI not valid"
                )

        # Remove all populations from BIBI.
        del bibi.brainModel.populations[:]

        if brain_populations is not None:
            for key, value in brain_populations.iteritems():
                population_node = None
                if isinstance(value, list):
                    population_node = bibi_api_gen.List()
                    for index in value:
                        population_node.append(index)
                if isinstance(value, dict):
                    population_node = bibi_api_gen.Range()
                    population_node.from_ = value['from']
                    population_node.to = value['to']
                    population_node.step = value.get('step')
                population_node.population = key
                bibi.brainModel.populations.append(population_node)

        if tempfile.gettempdir() in bibi_file_path:
            logger.debug(
                "removing the temporary bibi configuration file %s",
                bibi_file_path
            )
            shutil.rmtree(os.path.dirname(bibi_file_path))

        client.replace_file_content_in_collab(
            bibi.toxml("utf-8"),
            client.BIBI_CONFIGURATION_MIMETYPE,
            "recovered_bibi_configuration.xml"
        )

        return 200
