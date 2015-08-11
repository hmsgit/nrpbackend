"""
ROSCLEClient unit test
"""

import rospy
from hbp_nrp_backend.cle_interface.ROSCLEState import ROSCLEState
from hbp_nrp_backend.cle_interface import ROSCLEClient
from mock import patch, MagicMock
import unittest


__author__ = 'HBP NRP software team'


class TestROSCLEClient(unittest.TestCase):

    LOGGER_NAME = ROSCLEClient.__name__
    NUMBER_OF_SERVICE_PROXIES = 8

    def setUp(self):
        patcher = patch('rospy.ServiceProxy')
        self.addCleanup(patcher.stop)
        self.serviceProxyMock = patcher.start()
        self.serviceProxyMocks = [MagicMock() for _ in range(self.NUMBER_OF_SERVICE_PROXIES)]
        self.serviceProxyMock.side_effect = self.serviceProxyMocks

    def test_constructor(self):
        client = ROSCLEClient.ROSCLEClient(0)
        for mocks in self.serviceProxyMocks:
            mocks.wait_for_service.assert_called_with(timeout=client.ROS_SERVICE_TIMEOUT)

    def test_constructor_timeout(self):
        self.serviceProxyMocks[2].wait_for_service.side_effect = rospy.ROSException()     
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client = ROSCLEClient.ROSCLEClient(0)
            self.serviceProxyMocks[0].wait_for_service.assert_called_with(timeout=client.ROS_SERVICE_TIMEOUT)
            self.serviceProxyMocks[1].wait_for_service.assert_called_with(timeout=client.ROS_SERVICE_TIMEOUT)
            self.serviceProxyMocks[2].wait_for_service.assert_called_with(timeout=client.ROS_SERVICE_TIMEOUT)
        self.assertEqual(len(self.serviceProxyMocks[3].wait_for_service.mock_calls), 0)
        self.assertEqual(len(self.serviceProxyMocks[4].wait_for_service.mock_calls), 0)

    def test_start_pause_reset(self):
        client = ROSCLEClient.ROSCLEClient(0)
        client.start()
        self.serviceProxyMocks[0].assert_called_with() # make sure start is called
        # make sure no other services have been called
        self.assertEqual(len(self.serviceProxyMocks[1].mock_calls), 1) # pause
        self.assertEqual(len(self.serviceProxyMocks[2].mock_calls), 1) # stop
        self.assertEqual(len(self.serviceProxyMocks[3].mock_calls), 1) # reset
        self.assertEqual(len(self.serviceProxyMocks[4].mock_calls), 1) # state
        client.pause()
        self.serviceProxyMocks[1].assert_called_with()
        # make sure no other services have been called
        self.assertEqual(len(self.serviceProxyMocks[0].mock_calls), 2) # start
        self.assertEqual(len(self.serviceProxyMocks[2].mock_calls), 1) # stop
        self.assertEqual(len(self.serviceProxyMocks[3].mock_calls), 1) # reset
        self.assertEqual(len(self.serviceProxyMocks[4].mock_calls), 1) # state
        client.reset()
        # make sure no other services have been called
        self.serviceProxyMocks[3].assert_called_with()
        self.assertEqual(len(self.serviceProxyMocks[0].mock_calls), 2) # start
        self.assertEqual(len(self.serviceProxyMocks[1].mock_calls), 2) # pause
        self.assertEqual(len(self.serviceProxyMocks[2].mock_calls), 1) # stop
        self.assertEqual(len(self.serviceProxyMocks[4].mock_calls), 1) # state

    def test_stop(self):
        client = ROSCLEClient.ROSCLEClient(0)
        client.start()
        self.serviceProxyMocks[0].assert_called_with()
        client.stop()
        self.serviceProxyMocks[2].assert_called_with()
        # After a stop, nothing else can be called:
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client.start()
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client.pause()
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client.stop()
        with self.assertRaises(ROSCLEClient.ROSCLEClientException):
            client.reset()

        # get state can still answer (with a warning though)
        assert (ROSCLEState.STOPPED == client.get_simulation_state())

    def test_call_service(self):
        srv = MagicMock()
        srv.side_effect = rospy.ServiceException()
        client = ROSCLEClient.ROSCLEClient(0)
        self.assertRaises(Exception, client._ROSCLEClient__call_service, srv)

    def test_get_simulation_state(self):
        client = ROSCLEClient.ROSCLEClient(0)
        client._ROSCLEClient__valid = False
        client.get_simulation_state()

        client._ROSCLEClient__valid = True
        client._ROSCLEClient__cle_state = MagicMock()
        client.get_simulation_state()
        self.assertEquals(client._ROSCLEClient__cle_state.call_count, 1)

        client._ROSCLEClient__cle_state.side_effect = rospy.ServiceException()
        client.get_simulation_state()
        self.assertEquals(client._ROSCLEClient__cle_state.call_count, 2)

    def test_get_simulation_transfer_functions(self):
        client = ROSCLEClient.ROSCLEClient(0)
        client._ROSCLEClient__cle_get_transfer_functions = MagicMock()

        client._ROSCLEClient__valid = False
        client.get_simulation_transfer_functions()

        client._ROSCLEClient__valid = True
        client.get_simulation_transfer_functions()
        self.assertEquals(client._ROSCLEClient__cle_get_transfer_functions.call_count, 1)

        client._ROSCLEClient__cle_get_transfer_functions.side_effect = rospy.ServiceException()
        client.get_simulation_transfer_functions()
        self.assertEquals(client._ROSCLEClient__cle_get_transfer_functions.call_count, 2)

    def test_set_simulation_transfer_function(self):
        client = ROSCLEClient.ROSCLEClient(0)
        client._ROSCLEClient__cle_set_transfer_function = MagicMock()

        client._ROSCLEClient__valid = False
        client.set_simulation_transfer_function("tf_0", "def tf_0(): \n return 0")

        client._ROSCLEClient__valid = True
        client.set_simulation_transfer_function("tf_1", "def tf_1(): \n return 1")
        self.assertEquals(client._ROSCLEClient__cle_set_transfer_function.call_count, 1)

        client._ROSCLEClient__cle_set_transfer_function.side_effect = rospy.ServiceException()
        client.set_simulation_transfer_function("tf_2", "def tf_2(): \n return 2")
        self.assertEquals(client._ROSCLEClient__cle_set_transfer_function.call_count, 2)

    def test_delete_simulation_transfer_function(self):
        client = ROSCLEClient.ROSCLEClient(0)
        client._ROSCLEClient__cle_delete_transfer_function = MagicMock()

        client._ROSCLEClient__valid = False
        client.delete_simulation_transfer_function("tf_0")

        client._ROSCLEClient__valid = True
        client.delete_simulation_transfer_function("tf_1")
        self.assertEquals(client._ROSCLEClient__cle_delete_transfer_function.call_count, 1)

        client._ROSCLEClient__cle_delete_transfer_function.side_effect = rospy.ServiceException()
        client.delete_simulation_transfer_function("tf_2")
        self.assertEquals(client._ROSCLEClient__cle_delete_transfer_function.call_count, 2)

if __name__ == '__main__':
    unittest.main()
