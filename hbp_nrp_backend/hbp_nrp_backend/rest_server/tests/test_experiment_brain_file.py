"""
Unit tests for the experiment SDF services
"""
__author__ = 'Daniel Peppicelli'

import json
import shutil
from mock import patch
from hbp_nrp_backend.rest_server.tests import RestTest
import os
import json
import tempfile
from hbp_nrp_commons.generated import bibi_api_gen

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

        self.test_directory = os.path.split(__file__)[0]
        self.temp_directory = tempfile.mkdtemp()


    def test_experiment_brain_put_withpoplist(self):

        bibi_original_path = os.path.join(self.test_directory, "BIBI","bibi_1.xml")
        bibi_temp_path = os.path.join(self.temp_directory, "bibi_test.xml")
        bibi_remote_path = os.path.join("/collab_dir", "bibi_test.xml")
        shutil.copyfile(bibi_original_path, bibi_temp_path)
        with open(bibi_temp_path) as bibi_xml:
            bibi = bibi_api_gen.CreateFromDocument(bibi_xml.read())
        self.mock_collabClient_instance.clone_bibi_file_from_collab_context.return_value = bibi, bibi_temp_path, bibi_remote_path

        context_id = '123456'
        brain_populations = [{'to': 4, 'step': 1, 'from': 0, 'name': 'record'}]
        body = {'data': self.brain_model, 'additional_populations': brain_populations}
        response = self.client.put(
            '/experiment/' + context_id + '/brain',
            data=json.dumps(body)
        )
        self.assertEqual(response.status_code, 200)

        self.mock_collabClient_instance.find_and_replace_file_in_collab.assert_called_with(body['data'], self.mock_collabClient_instance.BRAIN_PYNN_MIMETYPE, "recovered_pynn_brain_model.py")

        save_string_to_file = self.mock_collabClient_instance.replace_file_content_in_collab

        arg1, arg2, arg3 = save_string_to_file.call_args_list[0][0]
        bibi = bibi_api_gen.CreateFromDocument(arg1)
        for population in bibi.brainModel.populations:
            if population.population == 'record':
                self.assertEqual(population.from_, 0)
                self.assertEqual(population.to, 4)
                self.assertEqual(population.step, 1)
            else:
                self.fail("Unexpected identifier: " + population.population)

    def test_experiment_brain_put(self):

        bibi_original_path = os.path.join(self.test_directory, "BIBI","bibi_1.xml")
        bibi_temp_path = os.path.join(self.temp_directory, "bibi_test.xml")
        shutil.copyfile(bibi_original_path, bibi_temp_path)
        with open(bibi_temp_path) as bibi_xml:
            bibi = bibi_api_gen.CreateFromDocument(bibi_xml.read())
        self.mock_collabClient_instance.clone_bibi_file_from_collab_context.return_value = bibi, bibi_temp_path, "bibi_remote_path"

        context_id = '123456'
        brain_populations = {'index': [0], 'slice': {'from': 0, 'to': 12, 'step': 2}, 'list': [1, 2, 3]}
        body = {'data': self.brain_model, 'additional_populations': brain_populations}
        response = self.client.put(
            '/experiment/' + context_id + '/brain',
            data=json.dumps(body)
        )
        self.assertEqual(response.status_code, 200)

        self.mock_collabClient_instance.find_and_replace_file_in_collab.assert_called_with(body['data'], self.mock_collabClient_instance.BRAIN_PYNN_MIMETYPE, "recovered_pynn_brain_model.py")

        save_string_to_file = self.mock_collabClient_instance.replace_file_content_in_collab

        arg1, arg2, arg3 = save_string_to_file.call_args_list[0][0]
        self.assertEqual(arg3, "bibi_remote_path")
        bibi = bibi_api_gen.CreateFromDocument(arg1)
        for population in bibi.brainModel.populations:
            if population.population == 'index':
                self.assertEqual(population.element, [0])
            elif population.population == 'slice':
                self.assertEqual(population.from_, 0)
                self.assertEqual(population.to, 12)
                self.assertEqual(population.step, 2)
            elif population.population == 'list':
                self.assertEqual(population.element, [1, 2, 3])
            else:
                self.fail("Unexpected identifier: " + population.population)
