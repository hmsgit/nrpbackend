"""
This package contains the implementation of the REST server to control experiments
"""

__author__ = 'GeorgHinkel'

import os
from flask import Flask
from flask_restful import Api
from flask_restful_swagger import swagger


class NRPServicesExtendedApi(Api):
    """
    Extend Flask Restful error handling mechanism so that we can still use original Flask error
    handlers (defined in __ErrorHandlers.py)
    """

    def error_router(self, original_handler, e):
        """
        Route the error

        :param original_handler: Flask handler
        :param e: Error
        """
        return original_handler(e)


class ParamNames(object):
    """
    Constants to be used as parameter names
    """
    ROBOT_ID = 'robotId'
    ROBOT_REL_PATH = 'robotRelPath'
    ROBOT_ABS_PATH = 'robotAbsPath'
    ROBOT_POSE = 'robotPose'
    IS_CUSTOM = 'isCustom'


class ErrorMessages(object):
    """
    Definition of error strings
    """
    ERROR_SAVING_FILE_500 = "Error saving file"
    SERVER_ERROR_500 = "The query failed due to an internal server error"

    EXPERIMENT_NOT_FOUND_404 = "The experiment with the given ID was not found"
    EXPERIMENT_BIBI_FILE_NOT_FOUND_404 = "The experiment BIBI file was not found"
    EXPERIMENT_CONF_FILE_NOT_FOUND_404 = "The experiment configuration file was not found"
    SIMULATION_NOT_FOUND_404 = "The simulation with the given ID was not found"

    DUPLICATE_NAME_403 = "Name already exists"
    OPERATION_INVALID_IN_CURRENT_STATE_403 = "The operation is forbidden while the simulation is " \
                                             "in its current state"

    SIMULATION_PERMISSION_401 = "Insufficient permissions to apply changes. Operation only allowed"\
                                " by simulation owner"

    SOURCE_CODE_ERROR_400 = "The source code is invalid: [ERROR-MESSAGE]"
    ACTIVATION_ERROR_400 = "The (de-)activation of the Transfer Function has failed"
    ERROR_IN_BASE64_400 = "Error in base64: {0}"

    EXP_VARIABLE_ERROR = """Error on server: environment variable: \
        'NRP_EXPERIMENTS_DIRECTORY' is empty"""
    MOD_VARIABLE_ERROR = """Error on server: environment variable: \
        'NRP_MODELS_DIRECTORY' is empty"""
    MODEXP_VARIABLE_ERROR = """Error on server: environment variable: \
        'NRP_MODELS_DIRECTORY' or 'NRP_EXPERIMENTS_DIRECTORY' is empty"""


app = Flask(__name__, static_folder='')
api = swagger.docs(NRPServicesExtendedApi(app), apiVersion='0.1')

# Import REST APIs
# pylint: disable=W0401
import hbp_nrp_backend.rest_server.__ErrorHandlers

from hbp_nrp_backend.rest_server.__SimulationResources import SimulationResources
from hbp_nrp_backend.rest_server.__Health import Last24HoursErrorCheck, TotalErrorCheck
from hbp_nrp_backend.rest_server.__SimulationBrainFile import SimulationBrainFile
from hbp_nrp_backend.rest_server.__SimulationControl import SimulationControl, LightControl, \
    MaterialControl
from hbp_nrp_backend.rest_server.__SimulationResetStorage import SimulationResetStorage
from hbp_nrp_backend.rest_server.__SimulationService import SimulationService
from hbp_nrp_backend.rest_server.__SimulationState import SimulationState
from hbp_nrp_backend.rest_server.__SimulationTransferFunctions import SimulationTransferFunction, \
    SimulationTransferFunctions, SimulationTransferFunctionActivation
from hbp_nrp_backend.rest_server.__SimulationStateMachines import SimulationStateMachines, \
    SimulationStateMachine
from hbp_nrp_backend.rest_server.__Version import Version
from hbp_nrp_backend.rest_server.__WorldSDFService import WorldSDFService
from hbp_nrp_backend.rest_server.__SimulationPopulations import SimulationPopulations
from hbp_nrp_backend.rest_server.__SimulationStructuredTransferFunctions import \
    SimulationConvertStructuredTransferFunctionToRaw, \
    SimulationConvertRawToStructuredTransferFunction
from hbp_nrp_backend.rest_server.__SimulationTimeout import SimulationTimeout
from hbp_nrp_backend.rest_server.__SimulationTopics import SimulationTopics
from hbp_nrp_backend.rest_server.__SimulationRecorder import SimulationRecorder
from hbp_nrp_backend.rest_server.__SimulationResourcesCloner import SimulationResourcesCloner
from hbp_nrp_backend.rest_server.__SimulationRobot import SimulationRobots, SimulationRobot


# Register /simulation
api.add_resource(LightControl, '/simulation/<int:sim_id>/interaction/light')
api.add_resource(
    MaterialControl, '/simulation/<int:sim_id>/interaction/material_change')
api.add_resource(SimulationBrainFile, '/simulation/<int:sim_id>/brain')
api.add_resource(SimulationResources, '/simulation/<int:sim_id>/resources')
api.add_resource(SimulationControl, '/simulation/<int:sim_id>')
api.add_resource(SimulationPopulations, '/simulation/<int:sim_id>/populations')
api.add_resource(SimulationResetStorage,
                 '/simulation/<int:sim_id>/<string:experiment_id>/reset')
api.add_resource(SimulationService, '/simulation')
api.add_resource(SimulationState, '/simulation/<int:sim_id>/state')
api.add_resource(SimulationTimeout, '/simulation/<int:sim_id>/extend_timeout')
api.add_resource(SimulationStateMachine,
                 '/simulation/<int:sim_id>/state-machines/<string:state_machine_name>')
api.add_resource(SimulationStateMachines,
                 '/simulation/<int:sim_id>/state-machines')
api.add_resource(SimulationTransferFunction,
                 '/simulation/<int:sim_id>/transfer-functions/<string:transfer_function_name>')
api.add_resource(SimulationTransferFunctionActivation,
                 '/simulation/<int:sim_id>/transfer-functions/<string:transfer_function_name>/'
                 'activation/<string:activate>')
api.add_resource(SimulationTransferFunctions,
                 '/simulation/<int:sim_id>/transfer-functions')
api.add_resource(SimulationTopics, '/simulation/topics')
api.add_resource(SimulationRecorder, '/simulation/<int:sim_id>/recorder/<string:command>')
api.add_resource(SimulationResourcesCloner,
                 '/simulation/clone-resources-files')
api.add_resource(SimulationConvertStructuredTransferFunctionToRaw,
                 '/simulation/<int:sim_id>/convert-structured-tf-to-raw')
api.add_resource(SimulationConvertRawToStructuredTransferFunction,
                 '/simulation/<int:sim_id>/convert-raw-tf-to-structured')

api.add_resource(SimulationRobots, '/simulation/<int:sim_id>/robots')
api.add_resource(SimulationRobot, '/simulation/<int:sim_id>/robots/<string:robot_id>')

# This should not be on the /simulation path ... as it does not apply to a
# running simulation
api.add_resource(WorldSDFService, '/simulation/<int:sim_id>/sdf_world')

# Register /health
api.add_resource(TotalErrorCheck, '/health/errors')
api.add_resource(Last24HoursErrorCheck, '/health/errors-last-24h')

# Register /version
api.add_resource(Version, '/version')
