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
from hbp_nrp_backend.rest_server.__CollabContext import CollabContext
from hbp_nrp_backend.rest_server import app

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

        get_or_raise_collab_context_patcher = patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.get_or_raise_collab_context')
        self.addCleanup(get_or_raise_collab_context_patcher.stop)
        self.mock_get_or_raise_collab_context = get_or_raise_collab_context_patcher.start()

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
        self.assertTrue(any('folder_a/experiment_configuration.xml' in args for args in argslist))
        self.assertTrue(any('folder_a/virtual_room.sdf' in args for args in argslist))
        # Check the link has been flattened
        for (arg1, arg2, arg3) in argslist:
            self.assertTrue(os.path.exists(arg1))
            if (arg1 == 'folder_a/virtual_room.sdf'):
                self.assertEqual(arg3,'application/hbp-neurorobotics.sdf.world+xml')
            if (arg2 == 'folder_a/experiment_configuration.xml'):
                self.assertEqual(arg3,'application/hbp-neurorobotics+xml')
                with open(arg1) as exd_file:
                    experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())
                    self.assertEqual(experiment.environmentModel.src, 'virtual_room.sdf')

        kall = rmtree_mock.call_args
        args, kwargs = kall
        self.temporary_directory_to_clean.append(args[0])

    @patch('tempfile.mkdtemp')
    def test_clone_experiment_template_from_collab(self, mkdtemp_mock):
        tmp_dir = "/tmp/tmp_directory"
        mkdtemp_mock.return_value = tmp_dir
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        experiment_folder = "fake_experiment_folder"
        experiment_conf = "exdConf.xml"
        environment_conf = "model.sdf"
        self.mock_document_client_instance.get_path_by_id.return_value = experiment_folder
        self.mock_document_client_instance.listdir.return_value = [experiment_conf,environment_conf]
        self.mock_document_client_instance.get_standard_attr.side_effect = [{'_entityType':'file','_contentType':'application/hbp-neurorobotics+xml'},{'_entityType':'file','_contentType':'application/hbp-neurorobotics.sdf.world+xml'}]
        result = neurorobotic_collab_client.clone_experiment_template_from_collab('some_uuid',)
        self.assertEqual(self.mock_document_client_instance.download_file.call_count, 2)
        self.assertEqual(result, {'experiment_conf': os.path.join(tmp_dir, experiment_conf), 'environment_conf': os.path.join(tmp_dir, environment_conf)})

    @patch('tempfile.mkdtemp')
    def test_clone_experiment_template_from_collab_context(self, mkdtemp_mock):
        tmp_dir = "/tmp/tmp_directory"
        mkdtemp_mock.return_value = tmp_dir
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        experiment_folder = "fake_experiment_folder"
        experiment_conf = "exdConf.xml"
        environment_conf = "model.sdf"
        self.mock_get_or_raise_collab_context.return_value = CollabContext('context_id', os.path.splitext(experiment_conf)[0], 'folder_uuid')
        self.mock_document_client_instance.get_path_by_id.return_value = experiment_folder
        self.mock_document_client_instance.listdir.return_value = [experiment_conf, environment_conf]
        self.mock_document_client_instance.get_standard_attr.side_effect = [{'_entityType':'file','_contentType':'application/hbp-neurorobotics+xml'},{'_entityType':'file','_contentType':'application/hbp-neurorobotics.sdf.world+xml'}]
        result = neurorobotic_collab_client.clone_experiment_template_from_collab_context()
        self.assertEqual(self.mock_document_client_instance.download_file.call_count, 2)
        self.assertEqual(result, {'experiment_conf': os.path.join(tmp_dir, experiment_conf), 'environment_conf': os.path.join(tmp_dir, environment_conf)})

    def test_get_first_file_path_with_mimetype(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        experiment_folder = "/fake_collab/fake_experiment_folder"
        experiment_conf = "exdConf.xml"
        environment_conf = "model.sdf"
        default_filename = "default_file.txt"
        self.mock_get_or_raise_collab_context.return_value = CollabContext('context_id', os.path.splitext(experiment_conf)[0], 'folder_uuid')
        self.mock_document_client_instance.get_path_by_id.return_value = experiment_folder
        self.mock_document_client_instance.listdir.return_value = [experiment_conf, environment_conf]
        self.mock_document_client_instance.get_standard_attr.side_effect = [
            {'_entityType':'file','_contentType':'application/hbp-neurorobotics+xml'},
            {'_entityType':'file','_contentType':'application/hbp-neurorobotics.sdf.world+xml'},
            {'_entityType':'file','_contentType':'application/hbp-neurorobotics+xml'},
            {'_entityType':'file','_contentType':'application/hbp-neurorobotics.sdf.world+xml'}]
        result = neurorobotic_collab_client.get_first_file_path_with_mimetype("fake_mimetype", "default_file.txt")
        self.assertEqual(result, os.path.join(experiment_folder,default_filename))
        result = neurorobotic_collab_client.get_first_file_path_with_mimetype("application/hbp-neurorobotics.sdf.world+xml", "default_file.txt")
        self.assertEqual(result, os.path.join(experiment_folder,environment_conf))

    def test_save_string_to_file_in_collab(self):
        filepath = "/fake_collab/fake_experiment_folder/file.txt"
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        neurorobotic_collab_client.get_first_file_path_with_mimetype = MagicMock()
        neurorobotic_collab_client.get_first_file_path_with_mimetype.return_value = filepath
        self.mock_document_client_instance.exists.side_effect = [True]
        mimetype = "fake_mime_type"
        string_to_upload = "some_string"
        neurorobotic_collab_client.save_string_to_file_in_collab(string_to_upload, mimetype, "default_file_name.txt")
        self.assertEqual(self.mock_document_client_instance.exists.call_count, 1)
        self.assertEqual(self.mock_document_client_instance.remove.call_count, 1)
        self.assertEqual(self.mock_document_client_instance.upload_string.call_count, 1)
        argslist = [x[0] for x in self.mock_document_client_instance.upload_string.call_args_list]
        arg1, arg2, arg3 = argslist[0]
        self.assertEqual(arg1, string_to_upload)
        self.assertEqual(arg2, filepath)
        self.assertEqual(arg3, mimetype)