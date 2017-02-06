"""
Unit tests for the experiment SDF services
"""
__author__ = 'Daniel Peppicelli'

import tempfile
import shutil
import os
import json
from mock import patch
from hbp_nrp_backend.rest_server.tests import RestTest
from hbp_nrp_commons.generated import bibi_api_gen


class TestExperimentTransferFunctions(RestTest):

    def setUp(self):
        patch_CollabClient = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient')
        self.addCleanup(patch_CollabClient.stop)
        self.mock_CollabClient = patch_CollabClient.start()
        self.mock_collabClient_instance = self.mock_CollabClient.return_value
        self.tf1_name = 'left_wheel_neuron_rate_monitor'
        self.tf1 = "@nrp.MapSpikeSink('left_wheel_neuron_aaa', nrp.brain.actors[1], nrp.population_rate)\n" \
                   "@nrp.Neuron2Robot(Topic('/monitor/population_rate', cle_ros_msgs.msg.SpikeRate))\n" \
                   "def " + self.tf1_name + "(t, left_wheel_neuron):\n" \
                   "    return cle_ros_msgs.msg.SpikeRate(t, left_wheel_neuron.rate, " \
                   "'left_wheel_neuron_rate_monitor')\n"
        self.tf2_name = 'all_neurons_spike_monitor'
        self.tf2 = "@nrp.MapSpikeSink('all_neurons_bbb', nrp.brain.circuit[slice(0, 8, 1)], " \
                   "nrp.spike_recorder)\n" \
                   "@nrp.Neuron2Robot(Topic('/monitor/spike_recorder', cle_ros_msgs.msg.SpikeEvent))\n" \
                   "def " + self.tf2_name + "(t, all_neurons):\n" \
                   "    return monitoring.create_spike_recorder_message(t, 8, all_neurons.times, " \
                   "'all_neurons_spike_monitor')\n"
        self.test_directory = os.path.split(__file__)[0]
        self.temp_directory = tempfile.mkdtemp()

    def tearDown(self):
        if os.path.isdir(self.temp_directory):
            shutil.rmtree(self.temp_directory)

    def test_experiment_transfer_functions_put(self):
        context_id = '123456'
        data = {'transfer_functions': [self.tf1, self.tf2]}
        bibi_original_path = os.path.join(self.test_directory, "experiments", "experiment_data","bibi_1.bibi")
        bibi_remote_path = os.path.join("/collab_remote_path", "bibi_1.bibi")
        bibi_temp_path = os.path.join(self.temp_directory, "bibi_test.xml")
        shutil.copyfile(bibi_original_path, bibi_temp_path)
        with open(bibi_temp_path) as bibi_xml:
            bibi = bibi_api_gen.CreateFromDocument(bibi_xml.read())
        self.mock_collabClient_instance.clone_bibi_file_from_collab_context.return_value = bibi, bibi_temp_path, bibi_remote_path
        response = self.client.put('/experiment/' + context_id + '/transfer-functions', data=json.dumps(data))
        self.assertEqual(response.status_code, 200)

        replace_file_argslist = self.mock_collabClient_instance.replace_file_content_in_collab.call_args_list[-1]
        replace_file_arg1, replace_file_arg2, replace_file_arg3 = replace_file_argslist[0]
        print replace_file_argslist[0]
        bibi = bibi_api_gen.CreateFromDocument(replace_file_arg1)
        tfs_names = [name + ".py" for name in self.tf1_name,self.tf2_name]
        for tf in bibi.transferFunction:
            self.assertTrue(tf.src in tfs_names)

        write_file_argslist  = [x[0] for x in self.mock_collabClient_instance.replace_file_content_in_collab.write_file_with_content_in_collab]
        for write_file_arg1, write_file_arg2, write_file_arg3, write_file_arg4 in write_file_argslist:
            self.assertTrue(write_file_arg4 in tfs_names)
            self.assertTrue(write_file_arg1 in [self.tf1, self.tf2])
            self.assertEqual(write_file_arg2, "application/hbp-neurorobotics.tfs+python")
