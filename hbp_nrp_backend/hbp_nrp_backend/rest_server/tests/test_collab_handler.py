"""
Unit tests for the service stores and retrives
key-value pairs of the form (Collab context UUID, experiment ID)
"""

__author__ = 'LucGuyot'

import unittest
import json
from mock import MagicMock, patch
from hbp_nrp_backend.rest_server.tests import RestTest
from sqlalchemy import exc
from hbp_nrp_backend.rest_server.__CollabContext import get_or_raise
from hbp_nrp_backend.rest_server import NRPServicesDatabaseException


class TestCollabHandler(RestTest):

    def setUp(self):
        patch_db = patch('hbp_nrp_backend.rest_server.__CollabHandler.db')
        self.mock_db = patch_db.start()
        self.addCleanup(patch_db.stop)

        patch_get_or_raise_collab_context = patch('hbp_nrp_backend.rest_server.__CollabHandler.get_or_raise_collab_context')
        self.addCleanup(patch_get_or_raise_collab_context.stop)
        self.mock_get_or_raise_collab_context = patch_get_or_raise_collab_context.start()
        self.exId = "ExDBraitenbergHuskySBC"
        self.uuId = "f81d4fae-7dec-11d0-a765-00a0c91e6bf6"
        self.experiment_folder_uuId = "/path/to/some/folder"
        self.mock_response_value = {'contextID': self.uuId, 'experimentID': self.exId, 'experimentFolderUUID': self.experiment_folder_uuId}
        self.mock_db_entry = MagicMock(context_id=self.uuId, experiment_id=self.exId, experiment_folder_uuid=self.experiment_folder_uuId)

        patch_CollabClient = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

        patch_get_experiment_conf = patch('hbp_nrp_backend.rest_server.__CollabHandler.get_experiment_conf')
        self.addCleanup(patch_get_experiment_conf.stop)
        self.mock_get_experiment_conf = patch_get_experiment_conf.start()

    def test_collab_handler_get(self):
        # The database contains the queried entry
        self.mock_get_or_raise_collab_context.return_value=self.mock_db_entry
        response = self.client.get('/collab/configuration/' + self.uuId)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data.strip(), json.dumps(self.mock_response_value))

        # The database has no entry with the given context UUID as primary key
        mock_response_value = {'contextID': self.uuId, 'experimentID': "", 'experimentFolderUUID': ""}
        self.mock_get_or_raise_collab_context.return_value=None
        response = self.client.get('/collab/configuration/' + self.uuId)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data.strip(), json.dumps(mock_response_value))

    def test_collab_handler_put(self):
        rqdata = {
            "experimentID": self.exId,
        }
        self.mock_get_or_raise_collab_context.return_value=None
        self.mock_collabClient_instance.clone_experiment_template_to_collab.return_value = self.experiment_folder_uuId
        response = self.client.put('/collab/configuration/' + self.uuId, data=json.dumps(rqdata))
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data.strip(), json.dumps(self.mock_response_value))

        error = None
        response = self.client.put('/collab/configuration/' + self.uuId, data=error)
        self.assertEqual(response.status_code, 400)

        # An entry with the given context UUID as primary key already exists
        self.mock_db.session = MagicMock(add=MagicMock(), commit=MagicMock())
        self.mock_get_or_raise_collab_context.return_value = self.mock_db_entry
        response = self.client.put('/collab/configuration/' + self.uuId, data=json.dumps(rqdata))
        self.assertEqual(response.status_code, 409)
        self.assertEqual(self.mock_db.session.add.call_count, 0)
        self.assertEqual(self.mock_db.session.commit.call_count, 0)

        # A new entry must be created in the database
        self.mock_get_or_raise_collab_context.return_value=None
        self.mock_db.session = MagicMock(add=MagicMock(), commit=MagicMock())
        response = self.client.put('/collab/configuration/' + self.uuId, data=json.dumps(rqdata))
        self.assertEqual(self.mock_db.session.add.call_count, 1)
        self.assertEqual(self.mock_db.session.commit.call_count, 1)

        # The template should have been cloned
        self.mock_collabClient_instance.clone_experiment_template_to_collab.assert_called()
if __name__ == '__main__':
    unittest.main()
