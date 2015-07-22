"""
This package contains the implementation of the REST server to control experiments
"""

__author__ = 'GeorgHinkel'


from flask import Flask
from flask_restful import Api
from flask_restful_swagger import swagger


class NRPServicesGeneralException(Exception):
    """
    General exception class that can be used to return meaningful messages
    to the ExD frontend.

    :param message: message displayed to the end user.
    :param error_type: Type of error (like 'CLE Error')
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
    def __init__(self, message, error_code):
        super(NRPServicesClientErrorException, self).__init__(message, 'Client error', error_code)


class NRPServicesStateException(NRPServicesGeneralException):
    """
    State exception class that can be used to return meaningful messages
    to the HBP frontend code.

    :param message: message displayed to the end user.
    :param error_type: Type of error (like 'CLE Error')
    """
    def __init__(self, message):
        super(NRPServicesStateException, self).__init__(message, "Transition error", 400)


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

# Import REST APIs
# pylint: disable=W0401
import hbp_nrp_backend.rest_server.__ErrorHandlers
from hbp_nrp_backend.rest_server.__SimulationService import SimulationService
from hbp_nrp_backend.rest_server.__SimulationState import SimulationState
from hbp_nrp_backend.rest_server.__SimulationControl import SimulationControl, LightControl, \
    CustomEventControl
from hbp_nrp_backend.rest_server.__SimulationTransferFunctions import SimulationTransferFunctions
from hbp_nrp_backend.rest_server.__WorldSDFService import WorldSDFService
from hbp_nrp_backend.rest_server.__Version import Version
from hbp_nrp_backend.rest_server.__ExperimentService import Experiment
from hbp_nrp_backend.rest_server.__ExperimentConf import ExperimentConf
from hbp_nrp_backend.rest_server.__ExperimentBibi import ExperimentBibi
from hbp_nrp_backend.rest_server.__ExperimentPreview import ExperimentPreview

api.add_resource(SimulationService, '/simulation')
api.add_resource(SimulationControl, '/simulation/<int:sim_id>')
api.add_resource(SimulationState, '/simulation/<int:sim_id>/state')
api.add_resource(CustomEventControl, '/simulation/<int:sim_id>/interaction')
api.add_resource(LightControl, '/simulation/<int:sim_id>/interaction/light')
api.add_resource(SimulationTransferFunctions, '/simulation/<int:sim_id>/transferfunctions')
api.add_resource(WorldSDFService, '/simulation/sdf_world')
api.add_resource(Version, '/version')
api.add_resource(Experiment, '/experiment')
api.add_resource(ExperimentConf, '/experiment/<string:exp_id>/conf')
api.add_resource(ExperimentBibi, '/experiment/<string:exp_id>/bibi')
api.add_resource(ExperimentPreview, '/experiment/<string:exp_id>/preview')
