"""
Test file for testing hbp_nrp_backend.simulation_control.Simulation
"""

__author__ = 'Stefan Deser'

import unittest
from mock import patch
from hbp_nrp_backend.simulation_control import Simulation


class TestSimulation(unittest.TestCase):

    def test_constructor(self):
        sim_id = 'some_sim_id'
        experiment_conf = 'some_exp_id'
        owner = 'some_owner'
        sim_gzserver_host = 'some_gzserver_host'
        self.__simulation = Simulation(sim_id, experiment_conf, None, owner, sim_gzserver_host)
