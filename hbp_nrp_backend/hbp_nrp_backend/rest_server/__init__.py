"""
This package contains the implementation of the REST server to control experiments
"""

__author__ = 'GeorgHinkel'


from flask import Flask
from flask_restful import Api
from flask_restful_swagger import swagger
from hbp_nrp_cle.cle.ROSCLEClient import ROSCLEClientException


class NRPServicesGeneralException(Exception):
    """
    General exception class that can be used to return meaningful messages
    to the HBP frontend code.

    :param message: message displayed to the end user.
    :param error_type: Type of error (like 'CLE Error')
    """
    def __init__(self, message, error_type, error_code=500):
        super(NRPServicesGeneralException, self).__init__(message)
        # This field is handled by the error handling HBP frontend code.
        self.error_type = error_type
        self.error_code = error_code

    def __str__(self):
        return "" + repr(self.message) + " (" + self.error_type + ")"


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


def rest_response(response_type, message, response_code):
    """
    Crafts a response for a REST API suitable for being shown in the frontend.

    :param response_type: Type for the response (Error, Warning, Info, Debug, ...)
    :param message: The message to be sent.
    :param response_code: HTTP response code.
    :return: A tuple suitable to be returned by flask (body, response code, headers).
    """
    return {'type': response_type, 'message': message, 'code': response_code}, response_code, None


def rest_error(message, response_code):
    """
    Crafts an error response for a REST API suitable for being shown in the frontend.

    :param message: The message to be sent.
    :param response_code: HTTP response code.
    :return: A tuple suitable to be returned by flask (body, response code, headers).
    """
    return rest_response('Error', message, response_code)


app = Flask(__name__)
api = swagger.docs(NRPServicesExtendedApi(app), apiVersion='0.1')

# Import REST APIs
# pylint: disable=W0401
import hbp_nrp_backend.rest_server.__ErrorHandlers
from hbp_nrp_backend.rest_server.__SimulationService import SimulationService
from hbp_nrp_backend.rest_server.__SimulationState import SimulationState
from hbp_nrp_backend.rest_server.__SimulationControl import SimulationControl, LightControl, \
    CustomEventControl
from hbp_nrp_backend.rest_server.__Version import Version

api.add_resource(SimulationService, '/simulation')
api.add_resource(SimulationControl, '/simulation/<int:sim_id>')
api.add_resource(SimulationState, '/simulation/<int:sim_id>/state')
api.add_resource(CustomEventControl, '/simulation/<int:sim_id>/interaction')
api.add_resource(LightControl, '/simulation/<int:sim_id>/interaction/light')
api.add_resource(Version, '/version')
