"""
Unit tests for the service that retrieves hbp_nrp_services and hbp_nrp_cle versions
"""

__author__ = 'AxelVonArnim, LucGuyot'

import hbp_nrp_backend
import hbp_nrp_cle
from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.rest_server import __Version
from mock import patch, MagicMock
import unittest
import json


class TestVersion(unittest.TestCase):

    @patch('hbp_nrp_backend.rest_server.__Version.rospy')
    def test_version_get(self, mocked_rospy):
        cle_version = str(hbp_nrp_cle.__version__)
        mocked_cle_version = type('obj', (object,), {'version': cle_version})
        mocked_get_version_service = MagicMock(return_value=mocked_cle_version, wait_for_service=MagicMock(return_value=None))
        mocked_rospy.ServiceProxy = MagicMock(return_value=mocked_get_version_service)
        client = app.test_client()
        response = client.get('/version')
        self.assertEqual(mocked_get_version_service.call_count, 1)
        self.assertEqual(response.status_code, 200)
        expected_response = {'hbp_nrp_cle': cle_version, 'hbp_nrp_backend': str(hbp_nrp_backend.__version__)}
        erd = json.dumps(expected_response)
        self.assertEqual(response.data, erd)

if __name__ == '__main__':
    unittest.main()