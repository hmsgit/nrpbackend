"""
This module contains the error handlers for the REST server
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.rest_server import app
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
    return "Internal server error: " + repr(error), 500
