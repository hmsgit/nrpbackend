"""
Unit tests for the simulation setup
"""
__author__ = 'UgoAlbanese'


import rospy
from hbp_nrp_backend.rest_server import app
import mock
import unittest
import json


class MockServiceResponse:
    def __init__(self):
        pass

    sdf_dump = '<sdf>Dummy</sdf>'


class TestWorldSDFService(unittest.TestCase):
    def setUp(self):
        self.client = app.test_client()

    @mock.patch('hbp_nrp_backend.rest_server.__WorldSDFService.rospy')
    def test_worldSDF_service_get(self, mckd_rospy):
        # setup the mocks
        mckd_rospy.wait_for_service = mock.MagicMock(return_value=None)

        mckd_rospy.ServiceProxy = mock.MagicMock(return_value=MockServiceResponse)

        # call the service
        response = self.client.get('/simulation/sdf_world')

        expected_response_data = json.dumps({'sdf': '<sdf>Dummy</sdf>'})

        # check the response
        self.assertEqual(response.status_code, 200)

        self.assertEqual(response.data.strip(), expected_response_data)

    @mock.patch('hbp_nrp_backend.rest_server.__WorldSDFService.rospy.wait_for_service')
    def test_worldSDF_service_wait_for_service_fail(self, mckd_wait_for_service):
        mckd_wait_for_service.side_effect = rospy.ROSException('Mocked ROSException')

        # call the service
        response = self.client.get('/simulation/sdf_world')

        # check the status code
        self.assertEqual(response.status_code, 500)

    @mock.patch('hbp_nrp_backend.rest_server.__WorldSDFService.rospy.wait_for_service')
    @mock.patch('hbp_nrp_backend.rest_server.__WorldSDFService.rospy.ServiceProxy')
    def test_worldSDF_service_serviceProxy_fail(self, mckd_ServiceProxy, _):
        mckd_ServiceProxy.return_value = mock.MagicMock(
            side_effect=rospy.ServiceException('Mocked ServiceException'))

        # call the service
        response = self.client.get('/simulation/sdf_world')

        # check the status code
        self.assertEqual(response.status_code, 400)

if __name__ == '__main__':
    unittest.main()
