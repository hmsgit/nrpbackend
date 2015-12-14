"""
This package contains the implementation of the REST server to control experiments
"""

__author__ = 'GeorgHinkel'

import os
from flask import Flask
from flask_restful import Api
from flask_restful_swagger import swagger
from flask_sqlalchemy import SQLAlchemy
import hbp_nrp_backend as backend
import multiprocessing

from hbp_nrp_backend import NRPServicesGeneralException, NRPServicesClientErrorException, \
    NRPServicesStateException, NRPServicesTransferFunctionException, \
    NRPServicesStateMachineException, NRPServicesWrongUserException, \
    NRPServicesUnavailableROSService


class NRPServicesDatabaseException(NRPServicesGeneralException):
    """
    Database exception class that can be used to return meaningful messages
    to the HBP frontend code. It is considered as a server error with status
    500 (internal error).

    :param message: message displayed to the end user.
    """
    def __init__(self, message):
        super(NRPServicesDatabaseException, self).__init__(message, "Database error")


class NRPServicesDatabaseTimeoutException(NRPServicesDatabaseException):
    """
    Database exception class that can be used in the case when the database
    is not reachable (connection timeout)
    """
    def __init__(self):
        super(NRPServicesDatabaseTimeoutException, self).__init__("Database connection timeout")


class NRPServicesExtendedApi(Api):
    """
    Extend Flask Restful error handling mechanism so that we
    can still use original Flask error handlers (defined in
    __ErrorHandlers.py)
    """
    def error_router(self, original_handler, e):
        """
        Route the error

        :param original_handler: Flask handler
        :param e: Error
        """
        return original_handler(e)


app = Flask(__name__)
api = swagger.docs(NRPServicesExtendedApi(app), apiVersion='0.1')
db = SQLAlchemy(app)

# Import REST APIs
# pylint: disable=W0401
import hbp_nrp_backend.rest_server.__ErrorHandlers

from hbp_nrp_backend import hbp_nrp_backend_config
from hbp_nrp_backend.rest_server.__CollabHandler import CollabHandler
from hbp_nrp_backend.rest_server.__ExperimentBibi import ExperimentBibi
from hbp_nrp_backend.rest_server.__ExperimentConf import ExperimentConf
from hbp_nrp_backend.rest_server.__ExperimentBrainFile import ExperimentBrainFile
from hbp_nrp_backend.rest_server.__ExperimentPreview import ExperimentPreview
from hbp_nrp_backend.rest_server.__ExperimentService import Experiment
from hbp_nrp_backend.rest_server.__ExperimentStateMachines import ExperimentGetStateMachines, \
    ExperimentPutStateMachine
from hbp_nrp_backend.rest_server.__ExperimentTransferfunctions import ExperimentTransferfunctions
from hbp_nrp_backend.rest_server.__ExperimentWorldSDF import ExperimentWorldSDF
from hbp_nrp_backend.rest_server.__Health import Last24HoursErrorCheck, TotalErrorCheck
from hbp_nrp_backend.rest_server.__SimulationBrainFile import SimulationBrainFile
from hbp_nrp_backend.rest_server.__SimulationControl import SimulationControl, LightControl, \
    CustomEventControl
from hbp_nrp_backend.rest_server.__SimulationReset import SimulationReset
from hbp_nrp_backend.rest_server.__SimulationService import SimulationService
from hbp_nrp_backend.rest_server.__SimulationState import SimulationState
from hbp_nrp_backend.rest_server.__SimulationTransferFunction import SimulationTransferFunction
from hbp_nrp_backend.rest_server.__SimulationTransferFunctions import SimulationTransferFunctions
from hbp_nrp_backend.rest_server.__SimulationStateMachines import SimulationStateMachines, \
    SimulationStateMachine
from hbp_nrp_backend.rest_server.__Version import Version
from hbp_nrp_backend.rest_server.__WorldSDFService import WorldSDFService

api.add_resource(CollabHandler, '/collab/configuration/<string:context_id>')
api.add_resource(CustomEventControl, '/simulation/<int:sim_id>/interaction')
api.add_resource(Experiment, '/experiment')
api.add_resource(ExperimentConf, '/experiment/<string:exp_id>/conf')
api.add_resource(ExperimentBibi, '/experiment/<string:exp_id>/bibi')
api.add_resource(ExperimentBrainFile, '/experiment/<string:context_id>/brain')
api.add_resource(ExperimentGetStateMachines, '/experiment/<string:exp_id>/state-machines')
api.add_resource(ExperimentPreview, '/experiment/<string:exp_id>/preview')
api.add_resource(ExperimentPutStateMachine,
                 '/experiment/<string:exp_id>/state-machines/<string:state_machine_name>')
api.add_resource(ExperimentTransferfunctions, '/experiment/<string:context_id>/transfer-functions')
api.add_resource(ExperimentWorldSDF, '/experiment/<string:context_id>/sdf_world')
api.add_resource(Last24HoursErrorCheck, '/health/errors-last-24h')
api.add_resource(LightControl, '/simulation/<int:sim_id>/interaction/light')
api.add_resource(SimulationBrainFile, '/simulation/<int:sim_id>/brain')
api.add_resource(SimulationControl, '/simulation/<int:sim_id>')
api.add_resource(SimulationReset, '/simulation/<int:sim_id>/reset')
api.add_resource(SimulationService, '/simulation')
api.add_resource(SimulationState, '/simulation/<int:sim_id>/state')
api.add_resource(SimulationStateMachine,
                 '/simulation/<int:sim_id>/state-machines/<string:state_machine_name>')
api.add_resource(SimulationStateMachines, '/simulation/<int:sim_id>/state-machines')
api.add_resource(SimulationTransferFunction,
                 '/simulation/<int:sim_id>/transfer-functions/<string:transfer_function_name>')
api.add_resource(SimulationTransferFunctions, '/simulation/<int:sim_id>/transfer-functions')
api.add_resource(TotalErrorCheck, '/health/errors')
api.add_resource(Version, '/version')
api.add_resource(WorldSDFService, '/simulation/sdf_world')


def db_create_and_check(database, timeout=1):
    """
    Populates the database based on the configuration provided by the
    SQLAlchemy object and checks, if the database url can be reached
    within the defined timeout.
    :param database: SQLAlchemy database object to be addressed
    :param timeout: timeout for the connection (in s)
    :raises NRPServicesDatabaseTimeoutException: if the database
    connection attempt exceeds the timeout
    """

    db_proc = multiprocessing.Process(target=database.create_all)
    db_proc.start()
    db_proc.join(timeout)
    if db_proc.is_alive():
        db_proc.terminate()
        raise NRPServicesDatabaseTimeoutException()


def init():
    """
    General initialization. We do not want this to be done every time (especially when testing).
    This is why it has been put in a separate function.
    """

    app.config.from_object(backend.hbp_nrp_backend_config)
