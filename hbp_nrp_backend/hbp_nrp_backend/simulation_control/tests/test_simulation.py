"""
Test file for testing hbp_nrp_backend.simulation_control.Simulation
"""

__author__ = 'Stefan Deser'

import unittest
from os import path
from mock import patch, MagicMock, Mock
from hbp_nrp_backend.simulation_control import Simulation
from hbp_nrp_backend.simulation_control import __Simulation as sim_module


class TestSimulation(unittest.TestCase):

    def create_sm_mock(self, sm_id, sim_id, sm_path = None):
        sm = Mock()
        sm.sm_id = sm_id
        sm.sim_id = sim_id
        sm.sm_path = sm_path
        self.__simulation.state_machines.append(sm)

    def setUp(self):
        self.patch_state = patch('hbp_nrp_backend.simulation_control.__Simulation.Simulation.state')
        self.mock_state = self.patch_state.start()

        def create_state_machine_mock(id, sim_id):
            sm = Mock()
            sm.sm_id = id
            sm.sim_id = sim_id
            return sm

        self.original_smi = sim_module.StateMachineInstance
        sim_module.StateMachineInstance = create_state_machine_mock

        sim_id = 'some_sim_id'
        experiment_conf = 'some_exp_id'
        owner = 'some_owner'
        sim_gzserver_host = 'some_gzserver_host'
        self.__simulation = Simulation(sim_id, experiment_conf, None, owner, sim_gzserver_host)
        sm_path = path.join(path.split(__file__)[0], "sm_mock.py")
        self.create_sm_mock('SM1', 0, sm_path)
        self.create_sm_mock('SM2', 0)
        self.create_sm_mock('SM3', 0, sm_path)

        with open(sm_path, "r") as sm_file:
            self.valid_sm_code = sm_file.read()

    def tearDown(self):
        self.mock_state = self.patch_state.stop()
        sim_module.StateMachineInstance = self.original_smi

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
        self.assertIsNotNone(self.__simulation.state_machine_manager)
        code = self.__simulation.get_state_machine_code('SM2')
        self.assertFalse(code)
        code = self.__simulation.get_state_machine_code('SM3')
        self.assertEqual(self.valid_sm_code, code)
        response = self.__simulation.get_state_machine_code('non-existing-SM')
        self.assertFalse(response)

    def test_simulation_set_state_machine_code(self):
        another_valid_code = "import smach_ros\n" +\
            "from mock import Mock\n" +\
            "sm = Mock()"
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
        self.__simulation.state_machines[0].initialize_sm = Mock(side_effect=AttributeError)
        self.__simulation.state_machines[0].is_running = False
        invalid_code = "def spwan_state_machine():\n return None"
        self.assertRaises(
            AttributeError,
            self.__simulation.set_state_machine_code, 'SM1',
            invalid_code
        )

        # Creation of a new state machine
        response = self.__simulation.set_state_machine_code('SM4', another_valid_code)
        sm4_updated_code = self.__simulation.get_state_machine_code('SM4')
        self.assertEqual(another_valid_code, sm4_updated_code)

    def test_simulation_delete_state_machine(self):
        self.assertIsNotNone(self.__simulation.state_machine_manager)
        number_of_state_machines = len(self.__simulation.state_machines)
        self.__simulation.delete_state_machine('SM2')
        self.assertEqual(number_of_state_machines - 1, len(self.__simulation.state_machines))
        success, error_message = self.__simulation.delete_state_machine('SM2')
        self.assertFalse(success)
        self.assertTrue(error_message)

        number_of_state_machines = len(self.__simulation.state_machines)
        self.__simulation.delete_state_machine('SM1')
        self.__simulation.delete_state_machine('SM3')
        self.assertEqual(number_of_state_machines - 2, len(self.__simulation.state_machines))

        success, error_message = self.__simulation.delete_state_machine('SM1')
        self.assertFalse(success)
        self.assertTrue(error_message)


if __name__ == '__main__':
    unittest.main()
