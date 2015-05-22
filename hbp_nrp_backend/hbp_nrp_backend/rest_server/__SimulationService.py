"""
This module contains the REST services to setup the simulation
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server import api, NRPServicesGeneralException, rest_error
from hbp_nrp_backend.rest_server.__SimulationControl import \
    SimulationControl
from hbp_nrp_backend.rest_server.__UserAuthentication import \
    UserAuthentication
from flask import request
from flask_restful import Resource, fields, marshal, marshal_with
from flask_restful_swagger import swagger
import time
import socket
import os
import logging

# pylint: disable=R0201

logger = logging.getLogger(__name__)


class SimulationService(Resource):
    """
    The module to setup simulations
    """

    # rosbridge restart timeout in seconds
    ROSBRIDGE_TIMEOUT_SEC = 20

    # This method is introduced as a temporary bugfix for the "blue bar of death" [NRRPLT-1997]:
    # Sometimes rosbridge does not work properly after several simulations have been run.
    # Hence we restart it here, before a new simulation is created.
    # The restart via supervisorctl does not wait until the port is open. Therefore we do a busy
    # waiting and checking every second if the port of rosbridge (9090) is open.
    # After ROSBRIDGE_TIMEOUT_SEC seconds a RosbridgeRestartTimeoutException is raised.
    @staticmethod
    def restart_rosbridge():
        """
        Restarts rosbridge
        """
        # Restart rosbridge since this piece of software is quite unstable.
        os.system("supervisorctl restart rosbridge")
        # active wait for port to be open
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        sleep_count = 0
        while sock.connect_ex(('127.0.0.1', 9090)) != 0:

            if sleep_count < SimulationService.ROSBRIDGE_TIMEOUT_SEC:
                logger.debug("waiting for port 9090 to be open: [%d] ", sleep_count)
                time.sleep(1)
                sleep_count += 1
            else:  # timeout!
                raise RosbridgeRestartTimeoutException(
                    "rosbridge restart timed out", "rosbridge error")

    @swagger.model
    class _Experiment(object):
        """
        Represents a simulation ID object. Only used for swagger documentation
        """

        resource_fields = {
            'experimentID': fields.String(),
            'gzserverHost': fields.String()
        }
        required = ['experimentID']

    @swagger.operation(
        notes='This is the entry point for the NRP REST server since'
              ' this is where you actually create simulations',
        responseClass=Simulation.__name__,
        responseMessages=[
            {
                "code": 400,
                "message": "Experiment ID is not valid"
            },
            {
                "code": 401,
                "message": "gzserverHost is not valid"
            },
            {
                "code": 408,
                "message": "rosbridge restart timed out"
            },
            {
                "code": 201,
                "message": "Simulation created successfully"
            }
        ],
        parameters=[
            {
                "name": "body",
                "paramType": "body",
                "dataType": _Experiment.__name__,
                "required": True
            }
        ]
    )
    def post(self):
        """
        Creates a new simulation which is neither 'initialized' nor 'started'.

        :<json string experimentID: Path and name of the experiment configuration file
        :>json string owner: The simulation owner (Unified Portal user name or 'hbp-default')
        :>json string state: The current state of the simulation (always 'created')
        :>json integer simulationID: The id of the simulation (needed for further REST calls)
        :>json string experimentID: Path and name of the experiment configuration file
        :>json string creationDate: Date of creation of this simulation
        :>json string gzserverHost: The host where gzserver will be run: local for using the
        same machine of the backend, lugano to use a dedicated instance on the Lugano viz cluster.
        :status 400: Experiment ID is not valid
        :status 401: gzserverHost is not valid
        :status 402: Another simulation is already running on the server
        :status 408: rosbridge restart timed out
        :status 201: Simulation created successfully
        """

        body = request.get_json(force=True)
        sim_id = len(simulations)

        if 'experimentID' not in body:
            return rest_error('Experiment ID not given.', 400)

        if ('gzserverHost' in body) and (body.get('gzserverHost') not in ['local', 'lugano']):
            return rest_error('gazebo server host not given or invalid.', 401)

        if True in [s.state != 'stopped' for s in simulations]:
            return rest_error('Another simulation is already running on the server.', 402)

        sim_gzserver_host = body.get('gzserverHost', 'local')
        sim_owner = UserAuthentication.get_x_user_name_header(request)
        try:
            SimulationService.restart_rosbridge()
        except RosbridgeRestartTimeoutException:
            return rest_error('ROSbridge restart timed out.', 408)
        simulations.append(Simulation(sim_id, body['experimentID'], sim_owner, sim_gzserver_host))

        return marshal(simulations[sim_id], Simulation.resource_fields), 201, {
            'location': api.url_for(SimulationControl, sim_id=sim_id),
            'gzserverHost': sim_gzserver_host
        }

    @swagger.operation(
        notes='Gets the list of all simulations on the server,'
              ' no matter what state',
        responseClass=list.__name__,
        responseMessages=[
            {
                "code": 200,
                "message": "Simulations retrieved successfully"
            }
        ]
    )
    @marshal_with(Simulation.resource_fields)
    def get(self):
        """
        Gets the list of simulations on this server.

        :>jsonarr string owner: The simulation owner (Unified Portal user name or 'hbp-default')
        :>jsonarr string state: The current state of the simulation
        :>jsonarr integer simulationID: The id of the simulation (needed for further REST calls)
        :>jsonarr string experimentID: Path and name of the experiment configuration file
        :>jsonarr string gzserverHost: Denotes where the gzserver will run once the simulation is
        started, local for localhost, lugano for a remote execution on the Lugano viz cluster.
        :>json string creationDate: Date of creation of this simulation
        :status 200: Simulations retrieved successfully
        """
        return simulations, 200


class RosbridgeRestartTimeoutException(NRPServicesGeneralException):
    """
    Exception raised by restart_rosbridge() when the restart
    of rosbridge times out
    """
    pass
