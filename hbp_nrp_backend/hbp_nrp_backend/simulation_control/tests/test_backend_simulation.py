"""
This module tests the backend implementation of the simulation lifecycle
"""

from mock import Mock, patch
import unittest
from os import path
from hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle import BackendSimulationLifecycle
from hbp_nrp_backend import NRPServicesGeneralException
import datetime
import rospy

__author__ = 'Georg Hinkel'

class TestBackendSimulationLifecycle(unittest.TestCase):

    def setUp(self):
        self.simulation = Mock()

        self.simulation.sim_id = 42
        self.simulation.experiment_conf = "ExDXMLExample.xml"
        self.simulation.environment_conf = None
        self.simulation.context_id = None
        self.simulation.state_machines = []

        caller_id = patch("hbp_nrp_commons.simulation_lifecycle.get_caller_id", return_value="test_client")
        caller_id.start()
        self.addCleanup(caller_id.stop)

        factory_client = patch("hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle.ROSCLESimulationFactoryClient")
        self.factory_mock = factory_client.start()
        self.addCleanup(factory_client.stop)

        cle_factory_client = patch("hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle.ROSCLEClient")
        self.cle_factory_mock = cle_factory_client.start()
        self.addCleanup(cle_factory_client.stop)

        rospy_patch = patch("hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle.rospy")
        self.rospy_mock = rospy_patch.start()
        self.addCleanup(rospy_patch.stop)

        with patch("hbp_nrp_commons.simulation_lifecycle.Publisher"):
            with patch("hbp_nrp_commons.simulation_lifecycle.Subscriber"):
                self.lifecycle = BackendSimulationLifecycle(self.simulation)
                self.lifecycle.models_path = path.split(__file__)[0]

        self.assertEqual("", self.lifecycle.experiment_path)
        self.assertEqual("", self.lifecycle.simulation_root_folder)

    def test_backend_initialize_non_collab(self):
        self.lifecycle.initialize(Mock())

        # Assert state machines have been initialized
        self.assertTrue(self.simulation.state_machine_manager.add_all.called)
        self.assertTrue(self.simulation.state_machine_manager.initialize_all.called)

        # Assert Simulation server has been called
        self.assertTrue(self.factory_mock.called)

        # Assert the simulation will be killed eventually
        self.assertIsInstance(self.simulation.kill_datetime, datetime.datetime)

        self.assertEqual(self.lifecycle.simulation_root_folder, self.lifecycle.models_path)
        self.assertIsNotNone(self.lifecycle.experiment_path)

    def test_backend_initialize_state_machines(self):
        self.simulation.experiment_conf = "ExDXMLExampleWithStateMachines.xml"

        self.lifecycle.initialize(Mock())

        state_machines = self.simulation.state_machine_manager.add_all.call_args[0][0]

        self.assertEqual(2, len(state_machines))
        directory = path.split(__file__)[0]
        self.assertEqual(path.join(directory, "SM1.py"), state_machines["SM1"])
        self.assertEqual(path.join(directory, "SM2.py"), state_machines["SM2"])

    def test_backend_initialize_collab(self):
        self.simulation.context_id = "Foobar"
        self.simulation.experiment_conf = "ExDXMLExampleWithStateMachines.xml"
        directory = path.split(__file__)[0]
        with patch("hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient") as collab_client:
            with patch("hbp_nrp_backend.rest_server.__UserAuthentication.UserAuthentication") as user_auth:

                collab_paths = {
                    'experiment_conf': path.join(directory, "ExDXMLExampleWithStateMachines.xml"),
                    'environment_conf': "Neverland.sdf"
                }

                collab_client().clone_experiment_template_from_collab_context.return_value = collab_paths

                self.lifecycle.initialize(Mock())

                self.assertTrue(user_auth.get_header_token.called)
                self.assertEqual("Foobar", collab_client.call_args[0][1])
                self.assertTrue(collab_client().clone_experiment_template_from_collab_context)

        # Assert that the state machine experiment has been called
        state_machines = self.simulation.state_machine_manager.add_all.call_args[0][0]
        self.assertEqual(2, len(state_machines))

        self.assertIsNotNone(self.lifecycle.experiment_path)
        self.assertIsNotNone(self.lifecycle.simulation_root_folder)

    def test_backend_initialize_nonexisting_experiment(self):
        self.simulation.experiment_conf = "DoesNotExist.xml"
        self.assertRaises(NRPServicesGeneralException, self.lifecycle.initialize, Mock())

    def test_backend_initialize_noclecommunication(self):
        self.rospy_mock.ROSException = rospy.ROSException
        self.factory_mock.side_effect = rospy.ROSException
        self.assertRaises(NRPServicesGeneralException, self.lifecycle.initialize, Mock())

    def test_backend_start(self):
        self.lifecycle.start(Mock())

        # Assert state machines have been started
        self.assertTrue(self.simulation.state_machine_manager.start_all.called)

    def test_backend_start_state_machines_failed(self):
        self.simulation.state_machine_manager.start_all.side_effect = IOError
        self.lifecycle.start(Mock())

        # Assert no exception, but state machine manager was still called
        self.assertTrue(self.simulation.state_machine_manager.start_all.called)

    def test_backend_stop(self):
        self.lifecycle.stop(Mock())

        # Assert state machines have been terminated
        self.assertTrue(self.simulation.state_machine_manager.terminate_all.called)

        self.assertIsNone(self.simulation.kill_datetime)

    def test_backend_pause(self):
        self.lifecycle.pause(Mock())

        # Assert state machines have been terminated
        self.assertTrue(self.simulation.state_machine_manager.terminate_all.called)

    def test_backend_fail(self):
        self.lifecycle.fail(Mock())

        # Assert state machines have been terminated
        self.assertTrue(self.simulation.state_machine_manager.terminate_all.called)

        self.assertIsNone(self.simulation.kill_datetime)

    def test_backend_reset(self):
        self.lifecycle.reset(Mock())

        # Assert state machines have been terminated
        self.assertTrue(self.simulation.state_machine_manager.terminate_all.called)
