"""
This module contains REST services for handling simulations reset,
Using the Collab as storage
"""

__author__ = "Alessandro Ambrosano, Ugo Albanese, Georg Hinkel"

import os
import tempfile
import shutil
import logging
import json

from cle_ros_msgs import srv, msg
from flask import request
from flask_restful import Resource, fields
from flask_restful_swagger import swagger

from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import \
    generate_tf, import_referenced_python_tfs, correct_indentation

from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClientException

from hbp_nrp_backend.rest_server import NRPServicesGeneralException, \
    NRPServicesWrongUserException, NRPServicesClientErrorException
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_commons.generated import bibi_api_gen
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import get_all_neurons_as_dict

from hbp_nrp_cleserver.server.ROSCLESimulationFactory import get_experiment_data
from hbp_nrp_backend.rest_server import NRPServicesTransferFunctionException
from hbp_nrp_commons.generated import exp_conf_api_gen
from distutils.dir_util import copy_tree  # pylint: disable=F0401,E0611

logger = logging.getLogger(__name__)

# pylint: disable=no-self-use


class SimulationResetCollab(Resource):
    """
    This resource handles the reset of a simulation, forwarding all the reset requests to the
    respective CLE instance.
    """

    @swagger.model
    class ResetRequest(object):
        """
        Represents a request for the API implemented by SimulationResetCollab
        """

        resource_fields = {'resetType': fields.Integer}
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
                "code": 200,
                "message": "Success. The reset type requested was performed on the given "
                    "simulation."
            },
            {
                "code": 400,
                "message": "Invalid request, the JSON parameters are incorrect."
            },
            {
                "code": 401,
                "message": "Operation only allowed by simulation owner"
            },
            {
                "code": 404,
                "message": "The simulation was not found."
            },
            {
                "code": 500,
                "message": "Reset unsuccessful due to a server error, better specified by the"
                    "error message."
            }
        ]
    )
    def put(self, sim_id, context_id):
        """
        Calls the CLE for resetting a given simulation to the last saved state in the Collab.

        :param sim_id: The simulation ID.
        :param context_id: The collab context ID
        :>json resetType: the reset type the user wants to be performed, details about possible
            values are given in GazeboRosPackages/src/cle_ros_msgs/srv/ResetSimulation.srv
        :status 200: The requested reset was performed successfully.
        :status 400: Invalid request, the JSON parameters are incorrect.
        :status 401: Operation only allowed by simulation owner.
        :status 404: The simulation with the given ID was not found.
        :status 500: Reset unsuccessful due to a server error, better specified by the error
            message.
        """

        sim = _get_simulation_or_abort(sim_id)

        if not UserAuthentication.matches_x_user_name_header(request, sim.owner):
            raise NRPServicesWrongUserException()

        body = request.get_json(force=True)

        for par in SimulationResetCollab.ResetRequest.required:
            if par not in body:
                raise NRPServicesClientErrorException('Missing parameter %s' % (par, ))

        for par in body:
            if par not in SimulationResetCollab.ResetRequest.resource_fields:
                raise NRPServicesClientErrorException('Invalid parameter %s' % (par, ))

        reset_type = body.get('resetType')

        world_sdf, brain_file, populations = SimulationResetCollab\
            ._compute_payload(reset_type, context_id)
        try:
            rsr = srv.ResetSimulationRequest
            if reset_type == rsr.RESET_FULL:
                SimulationResetCollab.resetFromCollabAll(sim)
                sim.cle.reset(reset_type, world_sdf=world_sdf, brain_path=brain_file,
                              populations=populations)
            elif reset_type == rsr.RESET_BRAIN:
                SimulationResetCollab.resetBrain(sim)
            else:
                sim.cle.reset(reset_type, world_sdf=world_sdf, brain_path=brain_file,
                              populations=populations)

        except ROSCLEClientException as e:
            raise NRPServicesGeneralException(str(e), 'CLE error', 500)

        return {}, 200

    @staticmethod
    def resetFromCollabAll(simulation):
        """
        Reset states machines and transfer functions
        """

        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient import \
            NeuroroboticsCollabClient

        client = NeuroroboticsCollabClient(
            UserAuthentication.get_header_token(request), simulation.context_id)

        collab_paths = client.clone_experiment_template_from_collab_context()
        cloned_base_path = os.path.dirname(collab_paths['experiment_conf'])

        current_experiment_path = simulation.lifecycle.experiment_path
        current_base_path = os.path.dirname(current_experiment_path)

        copy_tree(cloned_base_path, current_base_path)
        if tempfile.gettempdir() in cloned_base_path:
            shutil.rmtree(cloned_base_path)

        exp_conf, bibi_conf = get_experiment_data(current_experiment_path)

        SimulationResetCollab.resetBrain(simulation)

        SimulationResetCollab.resetTransferFunctions(simulation,
                                                     bibi_conf,
                                                     current_base_path)
        SimulationResetCollab.resetStateMachines(simulation, exp_conf, current_base_path)

    @staticmethod
    def resetBrain(simulation):
        """
        Reset brain
        :param simulation: simulation object
        :param bibi_conf: BIBI conf
        :param base_path: base path of the experiment
        """

        brainPath, _, neurons_config = \
                        SimulationResetCollab._get_brain_info_from_collab(simulation.context_id)

      # Convert the populations to a JSON dictionary

        for (name, s) in neurons_config.iteritems():
            v = {}
            v['from'] = s.start
            v['to'] = s.stop
            if s.step <= 0:
                v['step'] = 1
            else:
                v['step'] = s.step

            neurons_config[name] = v

        neurons_config = json.dumps(neurons_config)

        with open(brainPath, 'r') as myfile:
            data = myfile.read()
            NOT_CHANGE_POPULATION = 2
            result = simulation.cle.set_simulation_brain('py', data, "text", neurons_config,
                                                         NOT_CHANGE_POPULATION)
            if result.error_message is not "":
                # Error in given brain
                return {'error_message': result.error_message,
                        'error_line': result.error_line,
                        'error_column': result.error_column,
                        'handle_population_change': result.handle_population_change}, 300

    @staticmethod
    def resetStateMachines(sim, experiment, sm_base_path):
        """
        Reset states machines
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
    def resetTransferFunctions(simulation, bibi_conf, base_path):
        """
        Reset transfer functions
        :param simulation: simulation object
        :param bibi_conf: BIBI conf
        :param base_path: base path of the experiment
        """

        import_referenced_python_tfs(bibi_conf, base_path)

        for tf in bibi_conf.transferFunction:
            tf_code = generate_tf(tf, bibi_conf)
            tf_code = correct_indentation(tf_code, 0)
            tf_code = tf_code.strip() + "\n"
            logger.info(" RESET TF: " + tf.name + "\n" + tf_code + '\n')

            error_message = simulation.cle.set_simulation_transfer_function(
                str(tf.name),
                str(tf_code)
            )
            if (error_message):
                raise NRPServicesTransferFunctionException(
                    "Transfer function patch failed: "
                    + str(error_message) + "\n"
                    + "Updated source:\n"
                    + str(tf_code)
                )

    @staticmethod
    def _compute_payload(reset_type, context_id):
        """
        Compute the payload corresponding to a certain reset type

        :param reset_type:
        :param context_id: the collab context_id needed by RESET_WORLD
        :return: the payload for the reset request: a tuple of (world_sdf, brain_path, populations)
        """

        rsr = srv.ResetSimulationRequest

        if reset_type == rsr.RESET_WORLD:
            return SimulationResetCollab._get_sdf_world_from_collab(context_id), None, None
        elif reset_type == rsr.RESET_BRAIN:
            brain, populations, _ = SimulationResetCollab._get_brain_info_from_collab(context_id)
            return None, brain, populations
        else:
            return None, None, None

    @staticmethod
    def _get_brain_info_from_collab(context_id):
        """
        Gathers from the collab the brain script and the populations by getting the BIBI
        configuration file.

        :param context_id: the UUID of the collab in which to look for the brain information
        :return: A tuple with the path to the brain file and a list of populations
        """

        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient

        header_token = UserAuthentication.get_header_token(request)
        client = NeuroroboticsCollabClient(header_token, context_id)

        brain_collab_path = client.get_first_file_path_with_mimetype(
            NeuroroboticsCollabClient.BRAIN_PYNN_MIMETYPE,
            NeuroroboticsCollabClient.BRAIN_PYNN_FILE_NAME,
            "brain.py")

        bibi_collab_path = client.get_first_file_path_with_mimetype(
            NeuroroboticsCollabClient.BIBI_CONFIGURATION_MIMETYPE,
            NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME,
            NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME
        )

        temp_directory = tempfile.mkdtemp()
        brain_dst_path = os.path.join(temp_directory, 'brain.py')

        client.download_file_from_collab(brain_collab_path, brain_dst_path)

        bibi = bibi_api_gen.CreateFromDocument(client.download_file_from_collab(bibi_collab_path))
        neurons_config = get_all_neurons_as_dict(bibi.brainModel.populations)

        neurons_config_clean = [
            SimulationResetCollab._get_experiment_population(name, v)
            for (name, v) in neurons_config.iteritems()
        ]

        return brain_dst_path, neurons_config_clean, neurons_config

    @staticmethod
    def _get_experiment_population(name, value):
        """
        Gets an ExperimentPopulation object for the given population

        :param name: The population name
        :param value: The value describing the population
        :return:
        """
        if value is None:
            return msg.ExperimentPopulationInfo(name=name, type=0, ids=[], start=0, stop=0, step=0)
        elif isinstance(value, slice):
            return msg.ExperimentPopulationInfo(name=name, type=1, ids=[], start=value.start,
                                                stop=value.stop, step=value.step)
        elif isinstance(value, list):
            return msg.ExperimentPopulationInfo(name=name, type=2, ids=value, start=0,
                                                stop=0, step=0)

    @staticmethod
    def _get_sdf_world_from_collab(context_id):
        """
        Download from the collab an sdf world file as a string.
        The file belongs to the collab identified by context_id

        :param context_id: the UUID of the collab in which to look for the world sdf
        :return: The content of the world sdf file as a string
        """

        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient

        header_token = UserAuthentication.get_header_token(request)

        client = NeuroroboticsCollabClient(header_token, context_id)

        file_path_world_sdf = client.get_first_file_path_with_mimetype(
            NeuroroboticsCollabClient.SDF_WORLD_MIMETYPE,
            NeuroroboticsCollabClient.SDF_WORLD_FILE_NAME,
            "recovered_world.sdf")
        world_sdf_string = client.download_file_from_collab(file_path_world_sdf)

        return world_sdf_string
