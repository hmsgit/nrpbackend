"""
Unit tests for the service stores and retrives
key-value pairs of the form (Collab context UUID, experiment ID)
"""

__author__ = 'LucGuyot'

import unittest
import json
from mock import MagicMock, patch
from hbp_nrp_backend.rest_server.tests import RestTest
from flask import abort


class TestCollabHandler(RestTest):

    def setUp(self):
        self.patch_db = patch('hbp_nrp_backend.rest_server.__CollabHandler.db')
        self.mock_db = self.patch_db.start()
        self.patch_CollabContext = patch('hbp_nrp_backend.rest_server.__CollabHandler.CollabContext')
        self.mock_CollabContext = self.patch_CollabContext.start()
        self.exId = "ExDBraitenbergHuskySBC"
        self.uuId = "f81d4fae-7dec-11d0-a765-00a0c91e6bf6"
        self.mock_response_value = {'contextID': self.uuId, 'experimentID': self.exId}

    def tearDown(self):
        self.patch_db.stop()
        self.patch_CollabContext.stop()

    def test_collab_handler_get(self):
        mock_db_entry = MagicMock(context_id=self.uuId, experiment_id=self.exId)
        self.mock_CollabContext.query = MagicMock(get_or_404=MagicMock(return_value=mock_db_entry))
        response = self.client.get('/collab/configuration/' + self.uuId)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data.strip(), json.dumps(self.mock_response_value))

        self.mock_CollabContext.query = MagicMock(get_or_404=MagicMock())
        self.mock_CollabContext.query.get_or_404.side_effect = lambda x: abort(404)
        response = self.client.get('/collab/configuration/' + self.uuId)
        self.assertEqual(response.status_code, 404)

    def test_collab_handler_put(self):
        rqdata = {
            "experimentID": self.exId,
        }
        response = self.client.put('/collab/configuration/' + self.uuId, data=json.dumps(rqdata))
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data.strip(), json.dumps(self.mock_response_value))

        error = None
        response = self.client.put('/collab/configuration/' + self.uuId, data=error)
        self.assertEqual(response.status_code, 400)


if __name__ == '__main__':
    unittest.main()
