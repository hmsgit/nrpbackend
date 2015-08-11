"""
This module contains the REST implementation
for retrieving the transfer
functions used during an experiment.
"""

import logging
from flask_restful_swagger import swagger
from flask_restful import Resource
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort

__author__ = 'LucGuyot, DanielPeppicelli'

# pylint: disable=no-self-use
# We need to define the methods like "get", "put", "post", ... in
# the class even though they could be static. This is because of the
# Swagger framework.

logger = logging.getLogger(__name__)


class SimulationTransferFunctions(Resource):
    """
    Expose the source code of the CLE transfer functions as a REST service.
    """

    def __init__(self):
        Resource.__init__(self)

    @swagger.operation(
        notes='Gets all transfer functions',
        responseClass=list.__name__,
        # We do not have any error status code. If something goes wrong
        # for one or several transfer functions, the associated string in
        # the returned array will be empty.
        responseMessages=[
            {
                "code": 200,
                "message": "Transfer functions retrieved successfully"
            }
        ]
    )
    def get(self, sim_id):
        """
        Gets all transfer functions
        (robot to neuron and neuron to robot) in an array of strings.

        :param sim_id: The simulation ID whose transfer functions are retrieved
        :status 200: Transfer functions retrieved successfully
        """

        simulation = _get_simulation_or_abort(sim_id)

        return simulation.cle.get_simulation_transfer_functions(), 200
