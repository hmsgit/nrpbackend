"""
This module contains the REST services used to check the
health of the servers (by a nagios compatible system).
"""

import logging
import string
import json
from flask_restful_swagger import swagger
from datetime import datetime, timedelta
from hbp_nrp_backend.simulation_control import simulations
from flask_restful import Resource

# pylint: disable=R0201

logger = logging.getLogger(__name__)


def _get_level(number_of_errors, number_of_simulations):
    """
    Helper function that returns a string containing CRITICAL, WARNING and OK.
    """
    if (number_of_errors <= 0):
        return "OK"
    elif (number_of_errors <= number_of_simulations / 2):
        return "WARNING"
    else:
        return "CRITICAL"


class Last24HoursErrorCheck(Resource):
    """
    Resource to monitor the number of errors that did happen in the last 24 hours.
    """
    @swagger.operation(
        notes='Get an Inciga2 status (OK, WARNING or ERROR) regarding the number of'
              ' errors that did happen in the last 24 hours',
        responseClass=string.__name__,
        responseMessages=[
            {
                "code": 200,
                "message": "Success"
            }
        ]
    )
    def get(self):
        """
        Get a nagios status regarding the number of errors that happened in the last
        24 hours.
        """
        has_simulation_been_run_in_the_last_24h = \
            lambda simulation: (datetime.utcnow() - simulation.creation_datetime) < timedelta(1)
        filtered_simulations = filter(has_simulation_been_run_in_the_last_24h, simulations)
        number_of_errors = sum(simulation.errors for simulation in filtered_simulations)
        number_of_simulations = len(filtered_simulations)
        status = json.dumps({'state': _get_level(number_of_errors, number_of_simulations),
                             'errors': number_of_errors,
                             'simulations': number_of_simulations})
        return status, 200


class TotalErrorCheck(Resource):
    """
    Resource to monitor the number of errors that did happen since the server started.
    """
    @swagger.operation(
        notes='Get an Inciga2 status (OK, WARNING or ERROR) regarding the number of'
              ' errors that did happen since the server has been started',
        responseClass=string.__name__,
        responseMessages=[
            {
                "code": 200,
                "message": "Success"
            }
        ]
    )
    def get(self):
        """
        Get a nagios status regarding the number of errors that happened since
        the server started.
        """
        number_of_errors = sum(simulation.errors for simulation in simulations)
        number_of_simulation = len(simulations)
        status = json.dumps({'state': _get_level(number_of_errors, number_of_simulation),
                             'errors': number_of_errors,
                             'simulations': number_of_simulation})
        return status, 200
