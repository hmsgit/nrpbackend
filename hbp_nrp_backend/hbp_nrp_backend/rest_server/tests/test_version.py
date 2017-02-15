"""
Unit tests for the service that retrieves python versions
"""

__author__ = 'AxelVonArnim, LucGuyot'

import hbp_nrp_backend
import hbp_nrp_cle
import hbp_nrp_cleserver
import hbp_nrp_commons
import hbp_nrp_excontrol
import hbp_nrp_music_xml
import hbp_nrp_music_interface

import unittest
import json
from mock import patch, MagicMock
from hbp_nrp_backend.rest_server.tests import RestTest


class TestVersion(RestTest):

    def test_version_get(self):
        response = self.client.get('/version')
        self.assertEqual(response.status_code, 200)
        expected_response = {'hbp_nrp_cle': str(hbp_nrp_cle.__version__), 'hbp_nrp_backend': str(hbp_nrp_backend.__version__),
                             'hbp_nrp_cleserver': str(hbp_nrp_cleserver.__version__), 'hbp_nrp_commons': str(hbp_nrp_commons.__version__),
                             'hbp_nrp_excontrol': str(hbp_nrp_excontrol.__version__), 'hbp_nrp_music_xml': str(hbp_nrp_music_xml.__version__),
                             'hbp_nrp_music_interface': str(hbp_nrp_music_interface.__version__)
                            }
        erd = json.dumps(expected_response)
        self.assertEqual(response.data.strip(), erd)

if __name__ == '__main__':
    unittest.main()
