"""
Unit tests for the simulation setup
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.rest_server import app, SimulationService
from hbp_nrp_backend.rest_server.__SimulationService import RosbridgeRestartTimeoutException
from hbp_nrp_backend.simulation_control import simulations
from mock import patch, MagicMock
import unittest
import json
import time
import datetime


class TestSimulationService(unittest.TestCase):

    def setUp(self):
        self.now = datetime.datetime.now()
        self.__restart_rosbridge_patcher = patch("hbp_nrp_backend.rest_server.SimulationService.restart_rosbridge")
        # Ensure that the patcher is cleaned up correctly even in exceptional cases
        self.addCleanup(self.__restart_rosbridge_patcher.stop)
        self.__mocked_restart_rosbridge = self.__restart_rosbridge_patcher.start()

    @patch('hbp_nrp_backend.simulation_control.__Simulation.datetime')
    def test_simulation_service_post(self, mocked_date_time):
        client = app.test_client()
        mocked_date_time.datetime = MagicMock()
        mocked_date_time.datetime.now = MagicMock(return_value=self.now)
        n = len(simulations)

        response = client.post('/simulation', data=json.dumps({
                                                               "experimentID": "MyExample.xml",
                                                               "gzserverHost": "local"
                                                              }))

        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.headers['Location'], 'http://localhost/simulation/' + str(n))
        expected_response_data = {
            'owner': "default-owner",
            'state': "created",
            'creationDate': self.now.isoformat(),
            'simulationID': n,
            'experimentID': "MyExample.xml",
            'gzserverHost': 'local',
            'left_screen_color': 'Gazebo/Blue',
            'right_screen_color': 'Gazebo/Blue'
        }
        erd = json.dumps(expected_response_data)
        self.assertEqual(response.data, erd)
        self.assertEqual(len(simulations), n + 1)
        simulation = simulations[n]
        self.assertEqual(simulation.experiment_id, 'MyExample.xml')


    def test_simulation_service_wrong_gzserver_host(self):
        client = app.test_client()

        wrong_server = "wrong_server"
        response = client.post('/simulation', data=json.dumps({
                                                               "experimentID": "MyExample.xml",
                                                               "gzserverHost": wrong_server
                                                              }))
        self.assertEqual(response.status_code, 401)


    @patch("hbp_nrp_backend.rest_server.__SimulationService.time.sleep")
    @patch("hbp_nrp_backend.rest_server.__SimulationService.socket.socket.connect_ex")
    @patch("hbp_nrp_backend.rest_server.__SimulationService.os.system")
    def test_simulation_service_restart_rosbridge(self, mocked_os_system, mocked_socket, mocked_time_sleep):
        # stop the mocking of restart_rosbridge started in setUp
        self.__restart_rosbridge_patcher.stop()

        # test that an exception is raised after the timeout
        mocked_socket.return_value = 1  # connection error

        self.assertRaises(RosbridgeRestartTimeoutException, SimulationService.restart_rosbridge)
        self.assertNotEqual(mocked_os_system.call_count, 0)
        self.assertNotEqual(mocked_socket.call_count, 0)

        # now test that, when the socket is connected, no exception is raised
        mocked_socket.return_value = 0  # connected

        self.assertIsNone(SimulationService.restart_rosbridge())

        # restore the mock so that other tests can use it
        self.__mocked_restart_rosbridge = self.__restart_rosbridge_patcher.start()


    def test_simulation_service_rosbridge_timeout_exception(self):

        # set the mock so to raise an exception when called
        self.__mocked_restart_rosbridge.side_effect =\
            RosbridgeRestartTimeoutException("rosbridge restart timed out!", "rosbridge error")

        client = app.test_client()

        response = client.post('/simulation', data=json.dumps({
                                                               "experimentID": "MyExample.xml",
                                                               "gzserverHost": "local"
                                                              }))

        self.assertTrue(self.__mocked_restart_rosbridge.called)
        self.assertEqual(response.status_code, 408)

        self.__mocked_restart_rosbridge.side_effect = None  # reset mock side_effects


    def test_simulation_service_get(self):
        client = app.test_client()
        n = len(simulations)
        number_of_new_simulations = 4

        for i in range(number_of_new_simulations):
            ex_id = 'MyExample%d.xml' % (n + i, )
            param = json.dumps({'experimentID': ex_id, 'gzserverHost': 'local'})
            client.post('/simulation', data=param)
        response = client.get('/simulation')

        self.assertEqual(response.status_code, 200)
        m = n + number_of_new_simulations
        self.assertEqual(len(simulations), m)
        simulation = simulations[m - 1]
        experimentID = 'MyExample' + str(m - 1) + '.xml'
        self.assertEqual(simulation.experiment_id, experimentID)

    def test_simulation_service_no_experiment_id(self):
        client = app.test_client()
        n = len(simulations)

        rqdata = {
            "experimentIDx": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), n)

    def test_simulation_service_wrong_method(self):
        client = app.test_client()
        n = len(simulations)

        rqdata = {
            "experimentID": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = client.put('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 405)
        self.assertEqual(len(simulations), n)

if __name__ == '__main__':
    unittest.main()
