"""
This package contains the implementation of the REST server to control experiments
"""

__author__ = 'GeorgHinkel'


import os
from flask import Flask
from flask_restful import Api
from flask_restful_swagger import swagger
from flask_sqlalchemy import SQLAlchemy


class NRPServicesGeneralException(Exception):
    """
    General exception class that can be used to return meaningful messages
    to the ExD frontend.

    :param message: message displayed to the end user.
    :param error_type: Type of error (like 'CLE Error')
    :param error_code: The HTTP error code to send to the frontend.
    """
    def __init__(self, message, error_type, error_code=500):
        super(NRPServicesGeneralException, self).__init__(message)
        # This field is handled by the error handling HBP frontend code.
        self.error_type = error_type
        self.error_code = error_code

    def __str__(self):
        return "{0} ({1})".format(repr(self.message), self.error_type)


class NRPServicesClientErrorException(NRPServicesGeneralException):
    """
    Exception class for client (4xx) errors. It can be used to return meaningful messages
    to the ExD frontend.

    :param message: message displayed to the end user.
    :param error_code: The HTTP error code to send to the frontend.
    """
    def __init__(self, message, error_type="Client error", error_code=400):
        super(NRPServicesClientErrorException, self).__init__(message, error_type, error_code)


class NRPServicesStateException(NRPServicesGeneralException):
    """
    State exception class that can be used to return meaningful messages
    to the HBP frontend code.

    :param message: message displayed to the end user.
    """
    def __init__(self, message):
        super(NRPServicesStateException, self).__init__(message, "Transition error", 400)


class NRPServicesTransferFunctionException(NRPServicesGeneralException):
    """
    Transfer function exception class that can be used to return meaningful messages
    to the HBP frontend in case source code updates fail.

    :param message: message displayed to the end user.
    """
    def __init__(self, message):
        super(NRPServicesTransferFunctionException, self).\
            __init__(message, "Transfer function error", 400)


class NRPServicesStateMachineException(NRPServicesClientErrorException):
    """
    State machine exception class that can be used to return meaningful messages
    to the HBP frontend in case source code updates fail.
    :param message: message displayed to the end user.
    """
    def __init__(self, message, error_code):
        super(NRPServicesStateMachineException, self).\
            __init__(message, "State machine error", error_code)


class NRPServicesWrongUserException(NRPServicesClientErrorException):
    """
    State machine exception class that can be used to return meaningful messages
    to the HBP frontend in case source code updates fail.
    :param message: message displayed to the end user.
    """
    def __init__(self):
        super(NRPServicesWrongUserException, self).\
            __init__(
                "You need to be the simulation owner to apply your changes",
                "Wrong user",
                401
            )


class NRPServicesUnavailableROSService(NRPServicesGeneralException):
    """
    Server error with status 500 (internal error) that a ROS service is unavailable.
    It can be used to return meaningful messages to the ExD frontend.

    :param message: message displayed to the end user. It contains the text of the
                    corresponding ROS exception
    """
    def __init__(self, message):
        super(NRPServicesUnavailableROSService, self).__init__(
            "ROS service not available: " + message,
            "Unavailable ROS service"
        )


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

# Import models
from hbp_nrp_backend.rest_server.__CollabContext import CollabContext

# Import REST APIs
# pylint: disable=W0401
import hbp_nrp_backend.rest_server.__ErrorHandlers
from hbp_nrp_backend.rest_server.__SimulationService import SimulationService
from hbp_nrp_backend.rest_server.__SimulationState import SimulationState
from hbp_nrp_backend.rest_server.__SimulationControl import SimulationControl, LightControl, \
    CustomEventControl
from hbp_nrp_backend.rest_server.__SimulationTransferFunctions import SimulationTransferFunctions
from hbp_nrp_backend.rest_server.__WorldSDFService import WorldSDFService
from hbp_nrp_backend.rest_server.__SimulationTransferFunction import SimulationTransferFunction
from hbp_nrp_backend.rest_server.__SimulationStateMachines import SimulationStateMachines, \
    SimulationStateMachine
from hbp_nrp_backend.rest_server.__Version import Version
from hbp_nrp_backend.rest_server.__ExperimentService import Experiment
from hbp_nrp_backend.rest_server.__ExperimentConf import ExperimentConf
from hbp_nrp_backend.rest_server.__ExperimentBibi import ExperimentBibi
from hbp_nrp_backend.rest_server.__ExperimentPreview import ExperimentPreview
from hbp_nrp_backend.rest_server.__ExperimentTransferfunctions import ExperimentTransferfunctions
from hbp_nrp_backend.rest_server.__ExperimentStateMachines import ExperimentGetStateMachines, \
 ExperimentPutStateMachine
from hbp_nrp_backend.rest_server.__CollabHandler import CollabHandler
from hbp_nrp_backend.rest_server.__ExperimentBrainFile import ExperimentBrainFile
from hbp_nrp_backend.rest_server.__Health import Last24HoursErrorCheck, TotalErrorCheck

api.add_resource(Last24HoursErrorCheck, '/health/errors-last-24h')
api.add_resource(TotalErrorCheck, '/health/errors')
api.add_resource(SimulationService, '/simulation')
api.add_resource(SimulationControl, '/simulation/<int:sim_id>')
api.add_resource(SimulationState, '/simulation/<int:sim_id>/state')
api.add_resource(CustomEventControl, '/simulation/<int:sim_id>/interaction')
api.add_resource(LightControl, '/simulation/<int:sim_id>/interaction/light')
api.add_resource(SimulationTransferFunctions, '/simulation/<int:sim_id>/transfer-functions')
api.add_resource(CollabHandler, '/collab/configuration')
api.add_resource(SimulationTransferFunction,
                 '/simulation/<int:sim_id>/transfer-functions/<string:transfer_function_name>')
api.add_resource(SimulationStateMachines, '/simulation/<int:sim_id>/state-machines')
api.add_resource(SimulationStateMachine,
                 '/simulation/<int:sim_id>/state-machines/<string:state_machine_name>')

api.add_resource(WorldSDFService, '/simulation/sdf_world')
api.add_resource(Version, '/version')

api.add_resource(Experiment, '/experiment')
api.add_resource(ExperimentConf, '/experiment/<string:exp_id>/conf')
api.add_resource(ExperimentBibi, '/experiment/<string:exp_id>/bibi')
api.add_resource(ExperimentTransferfunctions, '/experiment/<string:exp_id>/transfer-functions')
api.add_resource(ExperimentPreview, '/experiment/<string:exp_id>/preview')
api.add_resource(ExperimentGetStateMachines, '/experiment/<string:exp_id>/state-machines')
api.add_resource(ExperimentPutStateMachine,
                 '/experiment/<string:exp_id>/state-machines/<string:state_machine_name>')
api.add_resource(ExperimentBrainFile, '/experiment/<string:exp_id>/brain')


def init():
    """
    General initialization. We do not want this to be done every time (especially when testing).
    This is why it has been put in a separate function.
    """
    app.config.from_object(os.environ['APP_SETTINGS'])
