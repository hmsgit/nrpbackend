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
Each "Navigation Item" in the Collaboratory portal is identified by a context UUID.
This object contains all the neurorobotics associated fields.
"""
import sys
from hbp_nrp_backend.rest_server import db, NRPServicesDatabaseException
from sqlalchemy import exc


def get_or_raise(context_id):
    """
    Get the collab object (containing the experiment ID and the document services folder) from the
    database or raise an NRPServicesDatabaseException if the get query raised an exception.

    :param context_id: The Collab context UUID of Navigation Item's client
    :return: the experiment ID associated to the given context UUID
    """
    collab_context = None
    try:
        # pylint: disable=no-member
        collab_context = CollabContext.query.get(context_id)
    except exc.SQLAlchemyError:
        raise NRPServicesDatabaseException("The neurorobotics_collab database is not available"), \
            None, \
            sys.exc_info()[2]
    return collab_context


# pylint does not recognise members created by SQLAlchemy,
# see http://stackoverflow.com/questions/4061720/
# pylint: disable=no-member
class CollabContext(db.Model):
    """
    Neurorobotics collaboratory app instance associated fields
    """
    __tablename__ = 'collab_context'

    context_id = db.Column(db.String(), primary_key=True)
    experiment_id = db.Column(db.String())
    experiment_folder_uuid = db.Column(db.String())

    def __init__(self,
                 context_id,
                 experiment_id,
                 experiment_folder_uuid):

        super(CollabContext, self).__init__()
        self.context_id = context_id
        self.experiment_id = experiment_id
        self.experiment_folder_uuid = experiment_folder_uuid

    def __repr__(self):
        return '<id {}>'.format(self.context_id)
