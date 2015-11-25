"""
NeuroroboticsCollabClient unit test
"""

import inspect
import unittest
import shutil
import os
from mock import patch, MagicMock
from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient import NeuroroboticsCollabClient
from hbp_nrp_backend.exd_config.generated import exp_conf_api_gen


class TestNeuroroboticsCollabClient(unittest.TestCase):

    def setUp(self):
        oidc_client_patcher = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.BBPOIDCClient')
        self.addCleanup(oidc_client_patcher.stop)
        self.mock_oidc_client = oidc_client_patcher.start()

        collab_client_patcher = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.CollabClient')
        self.addCleanup(collab_client_patcher.stop)
        self.mock_collab_client = collab_client_patcher.start()
        self.mock_collab_client_instance = self.mock_collab_client.return_value

        document_client_patcher = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.DocumentClient')
        self.addCleanup(document_client_patcher.stop)
        self.mock_document_client = document_client_patcher.start()
        self.mock_document_client_instance = self.mock_document_client.return_value

        self.models_directory = os.path.join(os.path.dirname(inspect.getfile(self.__class__)), 'mock_model_folder')
        self.temporary_directory_to_clean = []

    def tearDown(self):
        for dir in self.temporary_directory_to_clean:
            if dir.startswith('/tmp'):
                shutil.rmtree(dir)
        self.temporary_directory_to_clean = []

    def test_get_context_app_name(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        self.mock_collab_client_instance.get_current_tree.return_value = {
            'children': [{'context': 'aaa', 'name': 'aaa_name'}, {'context': 'bbb', 'name': 'bbb_name'}]
        }
        self.assertEqual(neurorobotic_collab_client.get_context_app_name(), 'aaa_name')

    def test_generate_unique_folder_name(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        self.mock_document_client_instance.listdir.return_value = ['a','a_0','b_0']
        self.assertEqual(neurorobotic_collab_client.generate_unique_folder_name('a'),'a_1')
        self.assertEqual(neurorobotic_collab_client.generate_unique_folder_name('b'),'b')

    @patch('shutil.rmtree')
    def test_clone_experiment_template_to_collab(self, rmtree_mock):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        neurorobotic_collab_client.clone_experiment_template_to_collab('folder_a', os.path.join(self.models_directory, 'ExDConf', 'ExDXMLExample.xml'))
        self.mock_document_client_instance.mkdir.assert_called_with('folder_a')
        argslist = [x[0] for x in self.mock_document_client_instance.upload_file.call_args_list]
        self.assertTrue(any('folder_a/ExDXMLExample.xml' in args for args in argslist))
        self.assertTrue(any('folder_a/virtual_room.sdf' in args for args in argslist))
        # Check the link has been flattened
        for (arg1, arg2) in argslist:
            self.assertTrue(os.path.exists(arg1))
            if (arg2 == 'folder_a/ExDXMLExample.xml'):
                with open(arg1) as exd_file:
                    experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())
                    self.assertEqual(experiment.environmentModel.src, 'virtual_room.sdf')

        kall = rmtree_mock.call_args
        args, kwargs = kall
        self.temporary_directory_to_clean.append(args[0])


