# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains the REST implementation
for saving to the Collab storage
the content of the CSV recorders of the simulation.
"""

import string
import csv
from flask_restful_swagger import swagger
from flask_restful import Resource, request, fields
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

    @swagger.model
    class CsvRecorder(object):
        """
        Csv recorder
        """

        resource_fields = {
            'file': fields.String,
            'data': fields.List(fields.List(fields.String)),
        }
        required = ['file', 'data']

    @swagger.model
    class CsvRecorderList(object):
        """
        Csv recorder list
        """

        resource_fields = {
            'file': fields.String,
            'data': fields.List(fields.List(fields.String)),
        }
        required = ['file', 'data']

    @swagger.operation(
        notes="Gets the Simulation CSV recorders' content",
        responseClass=CsvRecorderList.__name__,
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation whose state shall be retrieved",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
        ],
        multiValuedResponse=True,
        responseMessages=[
            {
                "code": 500,
                "message": "Error when retrieving recorder files"
            },
            {
                "code": 404,
                "message": "The simulation was not found"
            },
            {
                "code": 200,
                "message": "Recorders content retrieved successfully"
            }
        ]
    )
    def get(self, sim_id):
        """
        Gets the simulation CSV recorders' content

        :param sim_id: The simulation ID whose transfer functions are retrieved will be saved
        :status 404: The simulation with the given ID was not found
        :status 200: Returns a list of recorded csv files and data
        """
        simulation = _get_simulation_or_abort(sim_id)
        if not UserAuthentication.matches_x_user_name_header(request, simulation.owner):
            raise NRPServicesWrongUserException()

        response = []
        csv_files = simulation.cle.get_simulation_CSV_recorders_files()
        if csv_files:
            for csv_file in csv_files:
                file_data = []
                response.append({"file": csv_file.name, "data": file_data})
                with open(csv_file.temporary_path) as csvfile:
                    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
                    for row in spamreader:
                        file_data.append(', '.join(row))

        return response, 200

    @swagger.operation(
        notes='Save the simulation CSV recorders\' content to the Collab storage',
        parameters=[
            {
                "name": "sim_id",
                "description": "The ID of the simulation to to have the csv saved",
                "required": True,
                "paramType": "path",
                "dataType": int.__name__
            },
        ],
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
    def put(self, sim_id):
        """
        Save the simulation CSV recorders' content to the Collab storage.

        :param sim_id: The simulation ID whose transfer functions are retrieved
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
            simulation.context_id
        )
        time_string = get_date_and_time_string()
        subfolder_name = string.join(['csv_records', time_string], '_')
        client.populate_subfolder_in_collab(subfolder_name, csv_files, 'text/csv')

        return 200
