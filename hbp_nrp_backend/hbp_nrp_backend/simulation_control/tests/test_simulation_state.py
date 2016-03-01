"""
Tests the state management
"""

from hbp_nrp_backend.simulation_control.tests import unit_tests as utc
from hbp_nrp_backend.simulation_control.__Simulation import Simulation, InvalidStateTransitionException
import unittest
from hbp_nrp_backend.simulation_control import __StateMachine as sm

__author__ = 'Georg Hinkel'

class FooBarException(Exception):
    def __init__(self, msg="FooBar"):
        super(FooBarException, self).__init__(msg)

class TestSimulationState(unittest.TestCase):
    def setUp(self):
        utc.use_unit_test_transitions()

    def tearDown(self):
        utc.use_production_transitions()

    def test_state_transition_successful(self):
        sim = Simulation(0, "foo", "foo", "me", None, state="created")
        sim.state = "initialized"
        self.assertEqual(0, utc.last_sim_id)
        self.assertEqual("initialize", utc.last_transition)
        self.assertEqual("initialized", sim.state)
        self.assertEqual(0, sim.errors)

    def test_state_transition_invalid(self):
        sim = Simulation(1, "foo", "foo", "me", None, state="created")
        new_state = "stopped"
        def set_state():
            sim.state = new_state
        self.assertRaises(InvalidStateTransitionException, set_state)
        self.assertEqual("created", sim.state)
        self.assertEqual(0, sim.errors)

        sim = Simulation(2, "foo", "foo", "me", None, state="created")
        new_state = "not existing"
        self.assertRaises(InvalidStateTransitionException, set_state)
        self.assertEqual("created", sim.state)
        self.assertEqual(0, sim.errors)

    def test_state_transition_exception_created(self):
        sim = Simulation(3, "foo", "foo", "me", None, state="created")
        new_state = "initialized"
        def raise_exception(x):
            raise FooBarException()
        sm.initialize_simulation = raise_exception
        def set_initialized():
            sim.state = new_state
        self.assertRaises(FooBarException, set_initialized)
        self.assertEqual(3, utc.last_sim_id)
        self.assertEqual("clean", utc.last_transition)
        self.assertEqual("failed", sim.state)
        self.assertEqual(1, sim.errors)

    def test_state_transition_exception_initialized(self):
        sim = Simulation(4, "foo", "foo", "me", None, state="initialized")
        new_state = "started"
        def raise_exception(x):
            raise FooBarException()
        sm.start_simulation = raise_exception
        def set_started():
            sim.state = new_state
        self.assertRaises(FooBarException, set_started)
        self.assertEqual(4, utc.last_sim_id)
        self.assertEqual("clean", utc.last_transition)
        self.assertEqual("halted", sim.state)
        self.assertEqual(1, sim.errors)

    def test_state_transition_cleanup_failed(self):
        sim = Simulation(5, "foo", "foo", "me", None, state="created")
        sim.counter = 0
        def raise_exception(x):
            sim.counter += 1
            if sim.counter == 1:
                raise FooBarException("Foo")
            else:
                raise FooBarException("Bar")
        sm.initialize_simulation = raise_exception
        sm.clean = raise_exception
        exception_raised = False
        try:
            sim.state = "initialized"
        except Exception as e:
            exception_raised = True
            msg = repr(e)
            self.assertIn("initialized", msg)
            self.assertIn("Foo", msg)
            self.assertIn("Bar", msg)
        self.assertTrue(exception_raised)
        self.assertEqual("failed", sim.state)
        self.assertEqual(2, sim.errors)

    def test_state_transition_reroute(self):
        sim = Simulation(6, "foo", "foo", "me", None, state="halted")
        sim.state = "stopped"
        self.assertEqual(6, utc.last_sim_id)
        self.assertEqual("stop", utc.last_transition)
        self.assertEqual("failed", sim.state)
        self.assertEqual(0, sim.errors)
