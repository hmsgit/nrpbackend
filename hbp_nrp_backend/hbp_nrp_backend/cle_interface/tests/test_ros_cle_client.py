"""
ROSCLEClient unit test
"""

import rospy
from hbp_nrp_backend.cle_interface.ROSCLEState import ROSCLEState
from hbp_nrp_backend.cle_interface import ROSCLEClient, \
    SERVICE_SIM_START_ID, SERVICE_SIM_PAUSE_ID, \
    SERVICE_SIM_STOP_ID, SERVICE_SIM_RESET_ID, SERVICE_SIM_STATE_ID, \
    SERVICE_GET_TRANSFER_FUNCTIONS, SERVICE_SET_TRANSFER_FUNCTION, \
    SERVICE_DELETE_TRANSFER_FUNCTION, SERVICE_SET_BRAIN, SERVICE_GET_BRAIN, \
    SERVICE_GET_POPULATIONS, SERVICE_GET_CSV_RECORDERS_FILES, \
    SERVICE_CLEAN_CSV_RECORDERS_FILES
from cle_ros_msgs.srv import DeleteTransferFunction, ResetSimulation, ResetSimulationRequest
from std_srvs.srv import Empty
from cle_ros_msgs.srv import GetSimulationState, GetTransferFunctions, SetTransferFunction, \
    DeleteTransferFunction, GetBrain, SetBrain
from mock import patch, MagicMock
import unittest

from cle_ros_msgs import srv

__author__ = 'HBP NRP software team'


class TestROSCLEClient(unittest.TestCase):

    LOGGER_NAME = ROSCLEClient.__name__
    NUMBER_OF_SERVICE_PROXIES = 13

    def setUp(self):
        patcher = patch('rospy.ServiceProxy')
        self.addCleanup(patcher.stop)
        self.serviceProxyMock = patcher.start()
        self.serviceProxyMocks = [MagicMock() for _ in range(self.NUMBER_OF_SERVICE_PROXIES)]
        self.serviceProxyMock.side_effect = self.serviceProxyMocks

    @patch('hbp_nrp_backend.cle_interface.ROSCLEClient.ROSCLEServiceWrapper')
    def test_roscleclient_constructor(self, service_wrapper_mock):
        client = ROSCLEClient.ROSCLEClient(0)
        listened_services = [x[0][0] for x in service_wrapper_mock.call_args_list]
        expected_services = [
            SERVICE_SIM_RESET_ID(0), SERVICE_GET_TRANSFER_FUNCTIONS(0),
            SERVICE_SET_TRANSFER_FUNCTION(0), SERVICE_DELETE_TRANSFER_FUNCTION(0),
            SERVICE_GET_BRAIN(0), SERVICE_SET_BRAIN(0), SERVICE_GET_POPULATIONS(0),
            SERVICE_GET_CSV_RECORDERS_FILES(0)
        ]
        self.assertNotIn(False, [x in listened_services for x in expected_services])

    @patch('hbp_nrp_backend.cle_interface.ROSCLEClient.rospy.ServiceProxy')
    def test_rosservicewrapper_constructor(self, service_proxy_mock):
        # Case when the service is correctly subscribed
        service_wrapper = ROSCLEClient.ROSCLEServiceWrapper('service_name', Empty, None)
        self.assertEquals('service_name', service_proxy_mock.call_args[0][0])
        self.assertIsInstance(service_proxy_mock.call_args[0][1], type(Empty))
        args, kwargs = service_wrapper.handler.wait_for_service.call_args_list[0]
        self.assertEquals(kwargs['timeout'], ROSCLEClient.ROSCLEServiceWrapper.ROS_SERVICE_TIMEOUT)

        # Case when the service timeouts
        service_proxy_mock.side_effect = rospy.ROSException()
        self.assertRaises(ROSCLEClient.ROSCLEClientException,
            ROSCLEClient.ROSCLEServiceWrapper, 'service_name', Empty, None)

    @patch('hbp_nrp_backend.cle_interface.ROSCLEClient.rospy.ServiceProxy')
    def test_rosservicewrapper_call(self, service_proxy_mock):
        client = ROSCLEClient.ROSCLEClient(0)

        # When everything is fine, the service should just be called
        service_wrapper = ROSCLEClient.ROSCLEServiceWrapper('service_name', Empty, client)
        service_wrapper()
        self.assertEquals(service_wrapper.handler.call_count, 1)

        # When something's wrong during the service call it should throw ROSCLEClientException
        service_wrapper = ROSCLEClient.ROSCLEServiceWrapper('service_name', Empty, client,
                                                            invalidate_on_failure=False)
        service_wrapper.handler.side_effect = rospy.ServiceException()

        self.assertRaises(ROSCLEClient.ROSCLEClientException, service_wrapper)

        service_wrapper.handler.side_effect = rospy.exceptions.ROSInterruptException()

        self.assertRaises(ROSCLEClient.ROSCLEClientException, service_wrapper)

        # and invalidate the ROSCLEClient if asked to
        service_wrapper = ROSCLEClient.ROSCLEServiceWrapper('service_name', Empty, client,
                                                            invalidate_on_failure=True)
        self.assertRaises(ROSCLEClient.ROSCLEClientException, service_wrapper)

    def test_get_simulation_transfer_functions(self):
        msg = GetTransferFunctions()
        msg.transfer_functions = ['some', 'source', 'code']

        client = ROSCLEClient.ROSCLEClient(0)

        client._ROSCLEClient__cle_get_transfer_functions = MagicMock(return_value=msg)
        self.assertEquals(client.get_simulation_transfer_functions(), msg.transfer_functions)

    def test_set_simulation_transfer_function(self):
        msg = SetTransferFunction()
        resp = msg._response_class(error_message="Some error message")

        client = ROSCLEClient.ROSCLEClient(0)

        client._ROSCLEClient__cle_set_transfer_function = MagicMock(return_value=resp)
        self.assertEquals(
            client.set_simulation_transfer_function("tf_1", "def tf_1(): \n return 1"),
            resp.error_message)

    def test_delete_simulation_transfer_function(self):
        msg = DeleteTransferFunction()
        resp = msg._response_class(success=True)

        client = ROSCLEClient.ROSCLEClient(0)

        client._ROSCLEClient__cle_delete_transfer_function = MagicMock(return_value=resp)
        self.assertEquals(client.delete_simulation_transfer_function("tf_1"), resp.success)

    def test_get_brain(self):
        msg = GetBrain()
        resp = msg._response_class(brain_data='the brain data', brain_type='the brain type',
            data_type='the data type')

        client = ROSCLEClient.ROSCLEClient(0)

        client._ROSCLEClient__cle_get_brain = MagicMock(return_value=resp)
        self.assertEquals(client.get_simulation_brain(), resp)

    def test_set_brain(self):
        msg = SetBrain()
        resp = msg._response_class(error_message='error message', error_line=-1, error_column=-1)

        client = ROSCLEClient.ROSCLEClient(0)

        change_population = srv.SetBrainRequest.DO_RENAME_POPULATION

        client._ROSCLEClient__cle_set_brain = MagicMock(return_value=resp)
        self.assertEquals(client.set_simulation_brain(
            'data', 'py', 'text', '{"population_1": 2}', change_population), resp)
        client._ROSCLEClient__cle_set_brain.assert_called_once_with(
            'data', 'py', 'text', '{"population_1": 2}', change_population)

if __name__ == '__main__':
    unittest.main()
