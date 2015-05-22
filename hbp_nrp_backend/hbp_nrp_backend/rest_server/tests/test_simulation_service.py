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
        del simulations[:]

        client = app.test_client()
        mocked_date_time.datetime = MagicMock()
        mocked_date_time.datetime.now = MagicMock(return_value=self.now)

        response = client.post('/simulation', data=json.dumps({
                                                               "experimentID": "MyExample.xml",
                                                               "gzserverHost": "local"
                                                              }))

        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.headers['Location'], 'http://localhost/simulation/0')
        expected_response_data = {
            'owner': "default-owner",
            'state': "created",
            'creationDate': self.now.isoformat(),
            'simulationID': 0,
            'experimentID': "MyExample.xml",
            'gzserverHost': 'local',
            'left_screen_color': 'Gazebo/Blue',
            'right_screen_color': 'Gazebo/Blue'
        }
        erd = json.dumps(expected_response_data)
        self.assertEqual(response.data, erd)
        self.assertEqual(len(simulations), 1)
        simulation = simulations[0]
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
        del simulations[:]
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
        del simulations[:]
        client = app.test_client()

        param = json.dumps({'experimentID': 'MyExample.xml', 'gzserverHost': 'local'})
        response = client.post('/simulation', data=param)
        self.assertEqual(response.status_code, 201)

        response = client.get('/simulation')

        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(simulations), 1)
        simulation = simulations[0]
        self.assertEqual(simulation.experiment_id, 'MyExample.xml')
        self.assertEqual(simulation.gzserver_host, 'local')

    def test_simulation_service_no_experiment_id(self):
        del simulations[:]

        client = app.test_client()

        rqdata = {
            "experimentIDx": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_wrong_gzserver_host(self):
        del simulations[:]
        client = app.test_client()
        rqdata = {
            "experimentID": "MyExample.xml",
            "gzserverHost": "locano"
        }
        response = client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 401)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_another_sim_running(self):
        del simulations[:]
        client = app.test_client()
        rqdata = {
            "experimentID": "MyExample.xml",
            "gzserverHost": "lugano"
        }
        s = MagicMock('hbp_nrp_backend.simulation_control.Simulation')()
        s.state = 'started'
        client.post('/simulation', data=json.dumps(rqdata))
        response = client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 402)

    def test_simulation_service_wrong_method(self):
        del simulations[:]
        client = app.test_client()
        rqdata = {
            "experimentID": "MyExample.xml",
            "gzserverHost": "local"
        }
        response = client.put('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 405)
        self.assertEqual(len(simulations), 0)

if __name__ == '__main__':
    unittest.main()
