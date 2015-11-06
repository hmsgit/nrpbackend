"""
Each "Navigation Item" in the Collaboratory portal is identified by a context UUID.
This object contains all the neurorobotics associated fields.
"""
from hbp_nrp_backend.rest_server import db


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

    def __init__(self, context_id, experiment_id):
        super(CollabContext, self).__init__()
        self.context_id = context_id
        self.experiment_id = experiment_id

    def __repr__(self):
        return '<id {}>'.format(self.context_id)
