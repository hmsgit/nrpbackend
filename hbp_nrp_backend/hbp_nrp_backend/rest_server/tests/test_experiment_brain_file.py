"""
Unit tests for the experiment SDF services
"""
__author__ = 'Daniel Peppicelli'

import json
from mock import patch
from hbp_nrp_backend.rest_server.tests import RestTest


class TestExperimentBrain(RestTest):

    def setUp(self):
        patch_CollabClient = patch(
            'hbp_nrp_backend.collab_interface.'
            'NeuroroboticsCollabClient.NeuroroboticsCollabClient'
        )
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value

        self.brain_model = """import hbp_nrp_cle.tf_framework as nrp
import logging
import pyNN.nest as sim
import numpy as np
from pyNN.nest import *


nest.SetKernelStatus({'dict_miss_is_error': False})
logger = logging.getLogger(__name__)

def create_brain():
    C_m = 25.0
    g_L = 2.5
    t_m = C_m / g_L

    SENSORPARAMS = {'a': 0.0,
                    'b': 0.0,
                    'delta_T': 0.0,
                    'tau_w': 10.0,
                    'v_spike': 0.0,
                    'cm': C_m,  # 25.0 or 0.025?
                    'v_rest': -60.5,
                    'tau_m': t_m,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -60.5,
                    'v_thresh': -60.0,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    GO_ON_PARAMS = {'a': 0.0,
                    'b': 0.0,
                    'delta_T': 0.0,
                    'tau_w': 10.0,
                    'v_spike': 0.0,
                    'cm': C_m,
                    'v_rest': -60.5,
                    'tau_m': t_m,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -61.6,
                    'v_thresh': -60.51,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    cells = sim.Population(8, sim.IF_cond_alpha)
    params = {'U': 1.0, 'tau_rec': 0.0, 'tau_facil': 0.0}
    syndynamics = sim.SynapseDynamics(fast=sim.TsodyksMarkramMechanism(**params))"""

    def test_experiment_brain_put(self):
        context_id = '123456'
        data = {'data': self.brain_model}
        response = self.client.put('/experiment/' + context_id + '/brain', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)
        save_string_to_file = self.mock_collabClient_instance.save_string_to_file_in_collab
        argslist = [
            x[0] for x in save_string_to_file.call_args_list
        ]
        arg1, arg2, arg3 = argslist[0]
        self.assertEqual(arg1, self.brain_model)
        self.assertEqual(arg3, "recovered_pynn_brain_model.py")
