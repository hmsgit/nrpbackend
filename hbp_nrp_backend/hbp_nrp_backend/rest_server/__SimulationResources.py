"""
This module contains the REST implementation
for retrieving the simulation resource file list
"""

from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.rest_server import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__ExperimentService import ErrorMessages
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server.RestSyncMiddleware import RestSyncMiddleware
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen

import os

# pylint: disable=no-self-use


@swagger.model
class SimulationResource(object):

    """
    Simulation resource
    Only used for swagger documentation
    """

    resource_fields = {
        'file': fields.String,
        'type': fields.String
    }

    required = ['file', 'type']


@swagger.model
@swagger.nested(resources=SimulationResource.__name__)
class SimulationResourceList(object):

    """
    Simulation resource list
    Only used for swagger documentation
    """

    resource_fields = {
        'resources': fields.List(fields.Nested(SimulationResource.resource_fields))
    }

    required = ['resources']


class SimulationResources(Resource):

    """
    The simulation resource files
    """

    @swagger.operation(
        notes='Get the simulation resource file list of a given simulation.',
        responseClass=SimulationResourceList.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation",
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
                "message": ErrorMessages.EXPERIMENT_CONF_FILE_NOT_FOUND_404
            },
            {
                "code": 404,
                "message": ErrorMessages.EXPERIMENT_BIBI_FILE_NOT_FOUND_404
            },
            {
                "code": 200,
                "message": "Success. The simulation resource files were retrieved"
            }
        ]
    )
    @RestSyncMiddleware.threadsafe
    def get(self, sim_id):
        """
        Gets simulation resource files of the experiment running for the simulation ID

        :param sim_id: The simulation ID
        :>json string resources: Resource files
        :status 404: The simulation with the given ID was not found
        :status 404: The simulation BIBI file was not found
        :status 200: Success. The simulation BIBI configuration files were retrieved
        """

        simulation = _get_simulation_or_abort(sim_id)

        experiment_file = simulation.lifecycle.experiment_path

        if not os.path.isfile(experiment_file):
            raise NRPServicesClientErrorException(
                ErrorMessages.EXPERIMENT_CONF_FILE_NOT_FOUND_404,
                error_code=404)
        with open(experiment_file) as exd_file:
            experiment_dom = exp_conf_api_gen.CreateFromDocument(
                exd_file.read())

        bibi_fullpath = os.path.join(
            simulation.lifecycle.simulation_root_folder, experiment_dom.bibiConf.src)

        if not os.path.isfile(bibi_fullpath):
            raise NRPServicesClientErrorException(
                ErrorMessages.EXPERIMENT_BIBI_FILE_NOT_FOUND_404,
                error_code=404)

        resources = []
        for conf in experiment_dom.configuration:
            resources.append({'file': conf.src, 'type': conf.type})

        with open(bibi_fullpath) as _file:
            bibi_dom = bibi_api_gen.CreateFromDocument(_file.read())

        for conf in bibi_dom.configuration:
            resources.append({'file': conf.src, 'type': conf.type})

        if simulation.context_id:
            for conf in resources:
                conf['file'] = os.path.join('/config-from-cloned-folder', os.path.basename(
                    simulation.lifecycle.simulation_root_folder), os.path.basename(conf['file']))
        else:
            for conf in resources:
                conf['file'] = os.path.join(
                    '/config-from-template-folder', conf['file'])

        return {'resources': resources}, 200
