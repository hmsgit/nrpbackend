"""
This module contains the REST implementation
for saving to the Collab storage
the content of the CSV recorders of the simulation.
"""

import string
from flask_restful_swagger import swagger
from flask_restful import Resource, request
from hbp_nrp_backend import get_date_and_time_string
from hbp_nrp_backend import NRPServicesWrongUserException
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort

__author__ = 'LucGuyot, DanielPeppicelli'

# pylint: disable=no-self-use
# We need to define the methods like "get", "put", "post", ... in
# the class even though they could be static. This is because of the
# Swagger framework.


class SimulationCSVRecorders(Resource):
    """
    Expose the source code of the CLE transfer functions as a REST service.
    """

    def __init__(self):
        Resource.__init__(self)

    @swagger.operation(
        notes='Save the simulation CSV recorders\' content to the Collab storage',
        responseMessages=[
            {
                "code": 500,
                "message": "Error when saving recorder files"
            },
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 200,
                "message": "Recorders content saved successfully into Collab storage"
            }
        ]
    )
    def put(self, sim_id, context_id):
        """
        Save the simulation CSV recorders' content to the Collab storage.

        :param sim_id: The simulation ID whose transfer functions are retrieved
        :param context_id: The context UUID of the Collab where the CSV recorders content
         will be saved
        :status 500: Error when saving files
        :status 404: The simulation with the given ID was not found
        :status 200: Success. Files saved into Collab storage.
        """

        simulation = _get_simulation_or_abort(sim_id)
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        csv_files = simulation.cle.get_simulation_CSV_recorders_files()

        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient

        client = NeuroroboticsCollabClient(
            UserAuthentication.get_header_token(request),
            context_id
        )
        time_string = get_date_and_time_string()
        subfolder_name = string.join(['csv_records', time_string], '_')
        client.populate_subfolder_in_collab(subfolder_name, csv_files, 'text/csv')

        return 200
