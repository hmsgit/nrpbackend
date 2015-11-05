"""
Unit tests for the simulation setup
"""
__author__ = 'UgoAlbanese'


import rospy
import mock
import unittest
import json
from hbp_nrp_backend.rest_server.tests import RestTest


class MockServiceResponse:
    def __init__(self):
        pass

    sdf_dump = '<sdf>Dummy</sdf>'


class TestWorldSDFService(RestTest):

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


    @mock.patch('hbp_nrp_backend.rest_server.__WorldSDFService.tempfile')
    def test_worldSDF_put(self, mckd_tempfile):
        # Case OK
        mckd_namedtmp_instance = mock.MagicMock()
        mckd_namedtmp_instance.name = "randomname"
        # For testing with the 'with' statement
        mckd_namedtmp_instance.__enter__ = mock.MagicMock(return_value=mckd_namedtmp_instance)
        mckd_tempfile.NamedTemporaryFile = mock.MagicMock(return_value=mckd_namedtmp_instance)
        to_save = "<sdf>Dummy</sdf>"
        response = self.client.put('/simulation/sdf_world', data=json.dumps({'sdf': to_save}))

        self.assertEquals(response.status_code, 200)
        self.assertEquals(mckd_namedtmp_instance.name, json.loads(response.data)['path'])
        self.assertEquals(mckd_namedtmp_instance.write.call_args[0][0], to_save)

        # Invalid XML
        to_save = "<unsdf>Dummy</nein>"
        response = self.client.put('/simulation/sdf_world', data=json.dumps({'sdf': to_save}))

        self.assertEquals(response.status_code, 400)

        # Exception while writing the file
        to_save = "<sdf>Dummy</sdf>"
        mckd_namedtmp_instance.write = mock.MagicMock(side_effect=Exception)
        response = self.client.put('/simulation/sdf_world', data=json.dumps({'sdf': to_save}))

        self.assertEquals(response.status_code, 500)

if __name__ == '__main__':
    unittest.main()
