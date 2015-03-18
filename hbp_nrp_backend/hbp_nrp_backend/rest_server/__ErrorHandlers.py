"""
This module contains the error handlers for the REST server.
For 500 errors, a json object understandable by the HBP
frontend libraries is returned.
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.rest_server import app, NRPServicesGeneralException
from hbp_nrp_cle.cle.ROSCLEClient import ROSCLEClientException
import json
# pylint: disable=W0613


@app.errorhandler(404)
def not_found(error):
    """
    Handles cases where the command was not found on the server
    :param error: The error object
    """
    return "The command you requested was not found on this server", 404


@app.errorhandler(500)
def internal_error(error):
    """
    Handles internal server errors
    :param error: The error object
    """
    message = "Internal server error: " + str(error)
    return json.dumps({'message': message,
                       'type': 'General error'}), 500


@app.errorhandler(ROSCLEClientException)
# pylint does not understand the way Flask let
# users redefine handlers for Exceptions.
# pylint: disable=E0102
def internal_error(error):
    """
    Handles ROSCLEClientException errors
    :param error: The error object
    """
    message = "Error while communicating with the CLE"
    message += " (" + str(error) + ")."
    return json.dumps({'message': message, 'type': 'CLE error'}), 500


@app.errorhandler(NRPServicesGeneralException)
# pylint: disable=E0102
def error2json(error):
    """
    Handles NRPServicesGeneralException errors
    :param error: The error object
    """
    return json.dumps({'message': error.message,
                       'type': error.error_type}), error.error_code
