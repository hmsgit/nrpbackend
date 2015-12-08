"""
Each "Navigation Item" in the Collaboratory portal is identified by a context UUID.
This object contains all the neurorobotics associated fields.
"""
import sys
from hbp_nrp_backend.rest_server import db, NRPServicesDatabaseException
from sqlalchemy import exc


def get_or_raise(context_id):
    """
    Get the collab object (containing the experiment ID and the document
    services folder) from the database or raise an NRPServicesDatabaseException
    if the get query raised an exception.

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
