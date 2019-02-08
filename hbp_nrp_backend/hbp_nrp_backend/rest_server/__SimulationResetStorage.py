# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains REST services for handling simulations reset,
Using the storage
"""

__author__ = "Alessandro Ambrosano, Ugo Albanese, Georg Hinkel, Manos Angelidis"

import os
import logging
import json

from cle_ros_msgs import srv, msg
from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClient, ROSCLEClientException
from hbp_nrp_backend import NRPServicesGeneralException, \
    NRPServicesWrongUserException, NRPServicesClientErrorException
from hbp_nrp_backend.rest_server import ErrorMessages
from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient

from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.__UserAuthentication import UserAuthentication
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import generate_tf, \
    import_referenced_python_tfs, correct_indentation, get_tf_name, get_all_neurons_as_dict

from hbp_nrp_cleserver.server.ROSCLESimulationFactory import get_experiment_data
from hbp_nrp_commons.bibi_functions import docstring_parameter
from hbp_nrp_commons.generated import exp_conf_api_gen, bibi_api_gen

logger = logging.getLogger(__name__)

# pylint: disable=no-self-use


class SimulationResetStorage(Resource):
    """
    This resource handles the reset of a simulation, forwarding all the reset requests to the
    respective CLE instance.
    """

    storage_client = StorageClient()

    @swagger.model
    class ResetRequest(object):
        """
        Represents a request for the API implemented by SimulationResetStorage
        """

        resource_fields = {
            'resetType': fields.Integer,
            'contextId': fields.String()
        }
        required = ['resetType']

    @swagger.operation(
        notes='Handles the reset of a given simulation.',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
            {
                "name": "body",
                "paramType": "body",
                "dataType": ResetRequest.__name__,
                "required": True
            }
        ],
        responseMessages=[
            {
                "code": 500,
                "message": ErrorMessages.SERVER_ERROR_500
            },
            {
                "code": 404,
                "message": ErrorMessages.SIMULATION_NOT_FOUND_404
            },
            {
                "code": 401,
                "message": ErrorMessages.SIMULATION_PERMISSION_401
            },
            {
                "code": 400,
                "message": "Invalid request, the JSON parameters are incorrect"
            },
            {
                "code": 200,
                "message": "Success. The reset type requested was performed on the given "
                           "simulation"
            }
        ]
    )
    @docstring_parameter(ErrorMessages.SERVER_ERROR_500,
                         ErrorMessages.SIMULATION_NOT_FOUND_404,
                         ErrorMessages.SIMULATION_PERMISSION_401)
    def put(self, sim_id, experiment_id):
        """
        Calls the CLE for resetting a given simulation to the last saved state in the storage.

        :param sim_id: The simulation ID.
        :param experiment_id: The experiment ID

        :> json resetType: the reset type the user wants to be performed, details about possible
                          values are given in
                          GazeboRosPackages/src/cle_ros_msgs/srv/ResetSimulation.srv

        :status 500: {0}
        :status 404: {1}
        :status 401: {2}
        :status 400: Invalid request, the JSON parameters are incorrect
        :status 200: The requested reset was performed successfully
        """
        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, sim.owner):
            raise NRPServicesWrongUserException()

        req_body = request.get_json(force=True)
        context_id = req_body.get('contextId', None)

        for missing_par in (par for par in self.ResetRequest.required if par not in req_body):
            raise NRPServicesClientErrorException('Missing parameter {}'.format(missing_par))

        for invalid_p in (par for par in req_body if par not in self.ResetRequest.resource_fields):
            raise NRPServicesClientErrorException('Invalid parameter {}'.format(invalid_p))

        reset_type = req_body.get('resetType')

        rsr = srv.ResetSimulationRequest

        try:
            if reset_type == rsr.RESET_ROBOT_POSE:
                sim.cle.reset(reset_type)
            elif reset_type == rsr.RESET_WORLD:
                sim.cle.reset(reset_type,
                              world_sdf=self._get_sdf_world_from_storage(experiment_id, context_id))
            elif reset_type == rsr.RESET_FULL:
                brain_path, populations, _ = \
                    self._get_brain_info_from_storage(experiment_id, context_id)
                self.reset_from_storage_all(sim, experiment_id, context_id)

                sim.cle.reset(reset_type,
                              world_sdf=self._get_sdf_world_from_storage(experiment_id, context_id),
                              brain_path=brain_path,
                              populations=populations)
            else:
                return {}, 400  # Other reset modes are unsupported

        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        return {}, 200

    @classmethod
    def reset_from_storage_all(cls, simulation, experiment_id, context_id):
        """
        Reset states machines and transfer functions

        :param: the simulation id
        :param: the experiment id
        :param: the context_id for collab based simulations
        """

        simulation_dir = cls.storage_client.get_simulation_directory()
        cls.storage_client.clear_temp_sim_directory()

        _ = cls.storage_client.clone_all_experiment_files(
            UserAuthentication.get_header_token(request),
            experiment_id)

        exp_conf, bibi_conf = get_experiment_data(simulation.lifecycle.experiment_path)

        cls.reset_brain(simulation, experiment_id, context_id)
        cls.reset_transfer_functions(simulation, bibi_conf, simulation_dir)
        cls.reset_state_machines(simulation, exp_conf, simulation_dir)

    @classmethod
    def reset_brain(cls, simulation, experiment_id, context_id):
        """
        Reset brain

        :param simulation: simulation object
        :param experiment_id: the related experiment id
        :param context_id: the context ID for collab based simulations
        """
        brain_path, _, neurons_config = cls._get_brain_info_from_storage(experiment_id, context_id)
        # Convert the populations to a JSON dictionary
        for name, s in neurons_config.iteritems():
            neurons_config[name] = {
                'from': s.start,
                'to': s.stop,
                'step': s.step if s.step > 0 else 1
            }

        with open(brain_path, 'r') as brain_file:
            result_set_brain = simulation.cle.set_simulation_brain(
                brain_type='py',
                data_type='text',
                data=brain_file.read())

            if result_set_brain is not None and\
                    result_set_brain.error_message is not "":
                # Error in given brain
                raise ROSCLEClientException(
                    '{error_message}, line:{error_line},'
                    ' column:{error_column}'
                    .format(**result_set_brain.__dict__))

            result_set_populations = simulation.cle.set_simulation_populations(
                brain_type='py',
                data_type='text',
                brain_populations=json.dumps(neurons_config),
                change_population=ROSCLEClient.ReplaceBehaviorEnum.NO_REPLACE)

            if result_set_populations is not None and\
                    result_set_populations.error_message is not "":
                # Error in given brain
                raise ROSCLEClientException(
                    '{error_message}, line:{error_line},'
                    ' column:{error_column}'
                    .format(**result_set_populations.__dict__))

    @staticmethod
    def reset_state_machines(sim, experiment, sm_base_path):
        """
        Reset states machines

        :param sim:
        :param experiment: experiment conf
        :param sm_base_path: base path of the experiment
        """

        sim.delete_all_state_machines()

        state_machine_paths = {}
        if experiment.experimentControl is not None:
            state_machine_paths.update({sm.id: os.path.join(sm_base_path, sm.src)
                                        for sm in
                                        experiment.experimentControl.stateMachine
                                        if isinstance(sm, exp_conf_api_gen.SMACHStateMachine)})

        if experiment.experimentEvaluation is not None:
            state_machine_paths.update({sm.id: os.path.join(sm_base_path, sm.src)
                                        for sm in
                                        experiment.experimentEvaluation.stateMachine
                                        if isinstance(sm, exp_conf_api_gen.SMACHStateMachine)})

        sim.state_machine_manager.add_all(state_machine_paths, sim.sim_id)
        sim.state_machine_manager.initialize_all()

    @staticmethod
    def reset_transfer_functions(simulation, bibi_conf, base_path):
        """
        Reset transfer functions

        :param simulation: simulation object
        :param bibi_conf: BIBI conf
        :param base_path: base path of the experiment
        """
        old_tfs, _ = simulation.cle.get_simulation_transfer_functions()

        for old_tf_name in (get_tf_name(old_tf) for old_tf in old_tfs):
            if old_tf_name is not None:  # ignore broken TFs
                simulation.cle.delete_simulation_transfer_function(old_tf_name)

        import_referenced_python_tfs(bibi_conf, base_path)

        for tf in bibi_conf.transferFunction:
            tf_code = '{}\n'.format(correct_indentation(generate_tf(tf), 0).strip())

            logger.debug(" RESET TF: {tf_name}\n{tf_code}\n"
                         .format(tf_name=tf.name, tf_code=tf_code))
            # adding original TFs from the bibi
            # do not check the error message.
            # CLE will handle also invalid TFs
            simulation.cle.add_simulation_transfer_function(str(tf_code))

    @classmethod
    def _get_brain_info_from_storage(cls, experiment_id, context_id):
        """
        Gathers from the storage the brain script and the populations by getting the BIBI
        configuration file.

        :param experiment_id: the id of the experiment in which to look for the brain information
        :param context_id: the context ID for collab based simulations
        :return: A tuple with the path to the brain file and a list of populations
        """
        del context_id  # Unused

        request_token = UserAuthentication.get_header_token(request)

        experiment_file = cls.storage_client.get_file(
            request_token, experiment_id, 'experiment_configuration.exc', by_name=True)

        bibi_filename = exp_conf_api_gen.CreateFromDocument(experiment_file).bibiConf.src

        # find the brain filename from the bibi
        bibi_file = cls.storage_client.get_file(
            request_token, experiment_id, bibi_filename, by_name=True)

        bibi_file_obj = bibi_api_gen.CreateFromDocument(bibi_file)
        brain_filename = os.path.basename(bibi_file_obj.brainModel.file)

        brain_filepath = cls.storage_client.clone_file(brain_filename, request_token, experiment_id)

        neurons_config = get_all_neurons_as_dict(bibi_file_obj.brainModel.populations)

        neurons_config_clean = [
            SimulationResetStorage._get_experiment_population(name, v)
            for (name, v) in neurons_config.iteritems()
        ]

        return brain_filepath, neurons_config_clean, neurons_config

    @staticmethod
    def _get_experiment_population(name, value):
        """
        Gets an ExperimentPopulation object for the given population

        :param name: The population name
        :param value: The value describing the population
        :return:
        """
        if value is None:
            return msg.ExperimentPopulationInfo(
                name=name, type=0, ids=[], start=0, stop=0, step=0)
        if isinstance(value, slice):
            return msg.ExperimentPopulationInfo(
                name=name, type=1, ids=[], start=value.start, stop=value.stop, step=value.step)
        if isinstance(value, list):
            return msg.ExperimentPopulationInfo(
                name=name, type=2, ids=value, start=0, stop=0, step=0)

    @classmethod
    def _get_sdf_world_from_storage(cls, experiment_id, context_id):
        """
        Download from the storage an sdf world file as a string.
        The file belongs to the experiment identified by experiment_id

        :param experiment_id: the ID of the experiment in which to look for the world sdf
        :param context_id: the context ID for collab based simulations
        :return: The content of the world sdf file
        """
        del context_id  # Unused

        request_token = UserAuthentication.get_header_token(request)

        # find the sdf filename from the .exc
        experiment_file = cls.storage_client.get_file(
            request_token, experiment_id, 'experiment_configuration.exc', by_name=True)

        world_file_name = exp_conf_api_gen.CreateFromDocument(experiment_file).environmentModel.src

        return cls.storage_client.get_file(
            request_token, experiment_id, world_file_name, by_name=True)
