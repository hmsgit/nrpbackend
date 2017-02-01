"""
This file contains the unit tests for the simulation topics service
"""

from flask import Response
from mock import patch, Mock, PropertyMock
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_backend.rest_server.__SimulationControl import _get_simulation_or_abort
from hbp_nrp_backend.simulation_control import simulations
import json
from transitions import MachineError

class TestSimulationTopics(RestTest):

    def test_topics_husky_available(self):
        self.assertTrue(self.__is_topic_available("/husky/camera", "sensor_msgs/msg/camera"))
        self.assertTrue(self.__is_topic_available("/husky/cmd_vel", "cmd_msgs/cmd_vel"))

    def test_gazebo_topics_filtered(self):
        self.assertFalse(self.__is_topic_available("/gazebo/joint_states", "gazebo_ros_msgs/msg/jointStates"))

    def test_clock_topic_filtered(self):
        self.assertFalse(self.__is_topic_available("/clock", "Time"))

    def test_monitor_topics_filtered(self):
        self.assertFalse(self.__is_topic_available("/monitor/spike_recorder", "ros_cle_msgs/msg/SpikeRecorder"))

    def test_internal_topics_filtered(self):
        self.assertFalse(self.__is_topic_available("/ros_cle_simulation/status", "ros_cle_msgs/Status"))
        self.assertFalse(self.__is_topic_available("/ros_cle_simulation/error", "ros_cle_msgs/CLEError"))

    def __is_topic_available(self, topic, topic_type):
        with patch("hbp_nrp_backend.rest_server.__SimulationTopics.master") as master_mock:
            master_mock.Master().getTopicTypes.return_value = [(topic, topic_type)]
            topic_response = self.client.get("/simulation/topics")
            self.assertEqual(200, topic_response.status_code)
            topics_list = json.loads(topic_response.data)['topics']
            if len(topics_list) == 1:
                self.assertEqual(topic, topics_list[0]['topic'])
                self.assertEqual(topic_type, topics_list[0]['topicType'])
                return True
            elif len(topics_list) == 0:
                return False
            else:
                raise Exception("Two many topics returned: " + repr(topics_list))