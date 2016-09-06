"""
This module tests the simulate routine of the ROSCLESimulationFactory class
"""

__author__ = 'Bernd Eckstein'


import unittest
import os

from hbp_nrp_cleserver.server.ROSCLESimulationFactory import ROSCLESimulationFactory
from mock import Mock, patch
from hbp_nrp_cle.mocks.robotsim import MockRobotControlAdapter, MockRobotCommunicationAdapter
from hbp_nrp_cle.robotsim.LocalGazebo import LocalGazeboServerInstance
PATH = os.getcwd()
if not os.path.exists("ExDConf"):
    PATH += "/hbp_nrp_cleserver/hbp_nrp_cleserver/tests/server"


class MockedServiceRequest(object):
    environment_file = "environment_file.sdf"
    exd_config_file = os.path.join(PATH, "ExDConf/ExDXMLExample.xml")
    gzserver_host = "local"
    brain_processes = 1
    sim_id = 0


LocalGazeboServerInstance.start = LocalGazeboServerInstance.stop = \
    LocalGazeboServerInstance.restart = Mock()


class MockedGazeboHelper(object):

    def load_gazebo_world_file(self, world):
        return {}, {}

    def __getattr__(self, x):
        return Mock()

@patch("hbp_nrp_cleserver.server.CLELauncher.ROSCLEServer", new=Mock())
@patch("hbp_nrp_cleserver.server.CLELauncher.RosControlAdapter", new=MockRobotControlAdapter)
@patch("hbp_nrp_cleserver.server.CLELauncher.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
@patch("hbp_nrp_cleserver.server.CLELauncher.nrp.config.active_node", new=Mock())
@patch("hbp_nrp_cleserver.server.ROSCLESimulationFactory.get_experiment_basepath",
    new=Mock(return_value=PATH)
)
@patch("hbp_nrp_cleserver.server.CLELauncher.get_experiment_basepath", new=Mock(return_value=PATH))
@patch("hbp_nrp_cleserver.server.CLELauncher.LuganoVizClusterGazebo",
       new=LocalGazeboServerInstance)
@patch("hbp_nrp_cleserver.server.CLELauncher.LocalGazeboBridgeInstance", new=Mock())
@patch("hbp_nrp_cleserver.server.CLELauncher.GazeboHelper", new=MockedGazeboHelper)
@patch("hbp_nrp_cle.cle.ClosedLoopEngine.GazeboHelper", new=MockedGazeboHelper)
class SimulationTestCase(unittest.TestCase):

    def test_simulation_local(self):

        factory = ROSCLESimulationFactory()
        factory.create_new_simulation(MockedServiceRequest())

        factory.simulation_terminate_event.wait()
        self.assertEqual(factory.simulation_count, 1)
        self.assertEqual(factory.failed_simulation_count, 0)

    def test_simulation_lugano(self):

        MockedServiceRequest.gzserver_host = 'lugano'

        factory = ROSCLESimulationFactory()
        factory.create_new_simulation(MockedServiceRequest())

        factory.simulation_terminate_event.wait()
        self.assertEqual(factory.simulation_count, 1)
        self.assertEqual(factory.failed_simulation_count, 0)


if __name__ == '__main__':
    unittest.main()
