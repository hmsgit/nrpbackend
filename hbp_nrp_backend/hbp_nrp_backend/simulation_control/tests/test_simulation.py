"""
Test file for testing hbp_nrp_backend.simulation_control.Simulation
"""

__author__ = 'Stefan Deser'

import unittest
from mock import patch, MagicMock
from hbp_nrp_backend.simulation_control import Simulation
from hbp_nrp_backend.experiment_control import ExperimentStateMachineInstance


class TestSimulation(unittest.TestCase):

    def setUp(self):
        self.patch_state = patch('hbp_nrp_backend.simulation_control.__Simulation.Simulation.state')
        self.mock_state = self.patch_state.start()
        self.patch_threading_event = patch(
            'hbp_nrp_backend.experiment_control.__ExperimentStateMachine.threading.Event'
        )
        self.mock_threading_event = self.patch_threading_event.start()
        self.patch_rospy = patch(
            'hbp_nrp_backend.exd_config.default_state_machine.rospy'
        )
        self.mock_rospy = self.patch_rospy.start()

        sim_id = 'some_sim_id'
        experiment_conf = 'some_exp_id'
        owner = 'some_owner'
        sim_gzserver_host = 'some_gzserver_host'
        self.__simulation = Simulation(sim_id, experiment_conf, None, owner, sim_gzserver_host)
        self.valid_sm_code = "def create_state_machine():\n return None"
        self.__simulation.state_machines = {
            'SM1': ExperimentStateMachineInstance('SM1', self.valid_sm_code),
            'SM2': ExperimentStateMachineInstance('SM2'),
            'SM3': ExperimentStateMachineInstance('SM3', self.valid_sm_code)
        }

    def tearDown(self):
        self.mock_state = self.patch_state.stop()
        self.mock_threading_event = self.patch_threading_event.stop()
        self.mock_rospy = self.patch_rospy.stop()

    def test_simulation_constructor(self):
        sim_id = 'some_sim_id'
        experiment_conf = 'some_exp_id'
        owner = 'some_owner'
        sim_gzserver_host = 'some_gzserver_host'
        self.__simulation = Simulation(sim_id, experiment_conf, None, owner, sim_gzserver_host, 'view')
        self.__simulation = Simulation(
            sim_id, experiment_conf, None, owner,
            sim_gzserver_host, 'view', 'created'
        )
        self.__simulation = Simulation(
            sim_id, experiment_conf, None, owner,
            sim_gzserver_host, 'view', 'initialized'
        )

    def test_simulation_get_state_machine_code(self):
        code = self.__simulation.get_state_machine_code('SM2')
        self.assertEqual(None, code)
        code = self.__simulation.get_state_machine_code('SM3')
        self.assertEqual(self.valid_sm_code, code)
        response = self.__simulation.get_state_machine_code('non-existing-SM')
        self.assertEqual(False, response)

    @patch('hbp_nrp_backend.exd_config.default_state_machine.smach.StateMachine')
    def test_simulation_set_state_machine_code(self, smach_sm_mock):
        sm = MagicMock()
        sm.register_termination_cb = MagicMock(return_value=None)
        smach_sm_mock = MagicMock(return_value=sm)
        another_valid_code = "import smach_ros\n" +\
            "from hbp_nrp_backend.exd_config.default_state_machine import DefaultStateMachine\n" +\
            "def create_state_machine():\n return DummyStateMachine()\n" +\
            "class DummyStateMachine(DefaultStateMachine):\n" +\
            "    def __init__(self):\n" +\
            "        super(DummyStateMachine, self).__init__()"
        # The simulation state needs to be 'initialized'
        self.assertRaises(
            AssertionError, self.__simulation.set_state_machine_code,
            'SM1',
            another_valid_code
        )

        self.__simulation.state = 'initialized'
        response = self.__simulation.set_state_machine_code('SM2', another_valid_code)
        sm2_updated_code = self.__simulation.get_state_machine_code('SM2')
        self.assertEqual(another_valid_code, sm2_updated_code)

        # The create method is mandatory in the state machine code
        invalid_code = "def spwan_state_machine():\n return None"
        self.assertRaises(
            AttributeError,
            self.__simulation.set_state_machine_code, 'SM1',
            "def spwan_state_machine():\n return None"
        )

        # Creation of a new state machine
        response = self.__simulation.set_state_machine_code('SM4', another_valid_code)
        sm4_updated_code = self.__simulation.get_state_machine_code('SM4')
        self.assertEqual(another_valid_code, sm4_updated_code)

    def test_simulation_delete_state_machine(self):
        number_of_state_machines = len(self.__simulation.state_machines)
        self.__simulation.delete_state_machine('SM2')
        self.assertEqual(number_of_state_machines - 1, len(self.__simulation.state_machines))
        success, error_message = self.__simulation.delete_state_machine('SM2')
        self.assertEqual(False, success)
        self.assertTrue(error_message)

        number_of_state_machines = len(self.__simulation.state_machines)
        self.__simulation.delete_state_machine('SM1')
        self.__simulation.delete_state_machine('SM3')
        self.assertEqual(number_of_state_machines - 2, len(self.__simulation.state_machines))

        success, error_message = self.__simulation.delete_state_machine('SM1')
        self.assertEqual(False, success)
        self.assertTrue(error_message)
