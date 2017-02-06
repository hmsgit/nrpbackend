"""
NeuroroboticsCollabClient unit test
"""

import inspect
import unittest
import shutil
import os
from mock import patch, MagicMock, mock_open
from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
    import NeuroroboticsCollabClient, _FlattenedExperimentDirectory
from hbp_nrp_backend.collab_interface import NRPServicesUploadException
from bbp_client.document_service.exceptions import DocException
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from hbp_nrp_backend.rest_server.__CollabContext import CollabContext
from hbp_nrp_backend.rest_server import app, NRPServicesGeneralException


class TestNeuroroboticsCollabClient(unittest.TestCase):

    def setUp(self):
        oidc_client_patcher = patch(
            'hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.BBPOIDCClient'
        )
        self.addCleanup(oidc_client_patcher.stop)
        self.mock_oidc_client = oidc_client_patcher.start()

        collab_client_patcher = patch(
            'hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.CollabClient'
        )
        self.addCleanup(collab_client_patcher.stop)
        self.mock_collab_client = collab_client_patcher.start()
        self.mock_collab_client_instance = self.mock_collab_client.return_value

        document_client_patcher = patch(
            'hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.DocumentClient'
        )
        self.addCleanup(document_client_patcher.stop)
        self.mock_document_client = document_client_patcher.start()
        self.mock_document_client_instance = self.mock_document_client.return_value

        get_or_raise_collab_context_patcher = patch(
            'hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.get_or_raise_collab_context'
        )
        self.addCleanup(get_or_raise_collab_context_patcher.stop)
        self.mock_get_or_raise_collab_context = get_or_raise_collab_context_patcher.start()

        self.models_directory = os.path.join(
            os.path.dirname(inspect.getfile(self.__class__)), 'mock_model_folder'
        )
        self.experiments_directory = os.path.join(
            os.path.dirname(inspect.getfile(self.__class__)), 'experiment_data'
        )
        self.temporary_directory_to_clean = []

    def tearDown(self):
        for dir in self.temporary_directory_to_clean:
            if dir.startswith('/tmp'):
                shutil.rmtree(dir)
        self.temporary_directory_to_clean = []

    def test_get_context_app_name(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        self.mock_collab_client_instance.get_current_tree.return_value = {
            'children': [
                {'context': 'aaa', 'name': 'aaa_name'},
                {'context': 'bbb', 'name': 'bbb_name'}
            ]
        }
        self.assertEqual(neurorobotic_collab_client.get_context_app_name(), 'aaa_name')

    def test_generate_unique_folder_name(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        self.mock_document_client_instance.listdir.return_value = ['a', 'a_0', 'b_0']
        self.assertEqual(neurorobotic_collab_client.generate_unique_folder_name('a'), 'a_1')
        self.assertEqual(neurorobotic_collab_client.generate_unique_folder_name('b'), 'b')

    @patch('shutil.rmtree')
    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.get_model_basepath')
    def test_clone_experiment_template_to_collab_success(self, get_model_basepath_mock, rmtree_mock):
        get_model_basepath_mock.return_value = self.models_directory
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        neurorobotic_collab_client.clone_experiment_template_to_collab(
            'folder_a',
            os.path.join(self.experiments_directory, 'ExDXMLExample.exc')
        )
        self.mock_document_client_instance.mkdir.assert_called_with('folder_a')
        argslist = [x[0] for x in self.mock_document_client_instance.upload_file.call_args_list]
        self.assertTrue(any('folder_a/experiment_configuration.exc' in args for args in argslist))
        self.assertTrue(any('folder_a/ExDXMLExample.png' in args for args in argslist))
        self.assertTrue(any('folder_a/virtual_room.sdf' in args for args in argslist))
        self.assertTrue(any('folder_a/model.sdf' in args for args in argslist))
        self.assertTrue(any('folder_a/bibi_configuration.bibi' in args for args in argslist))
        self.assertTrue(any('folder_a/braitenberg_husky_non_linear_twist.py' in args for args in argslist))
        self.assertTrue(any('folder_a/braitenberg_husky_linear_twist.py' in args for args in argslist))
        self.assertTrue(any('folder_a/my_brain.py' in args for args in argslist))
        # Check the external links has been flattened
        for (arg1, arg2, arg3) in argslist:
            self.assertTrue(os.path.exists(arg1))
            if (arg2 == 'folder_a/ExDXMLExample.png'):
                self.assertEqual(arg3, 'image/png')
            if (arg2 == 'folder_a/virtual_room.sdf'):
                self.assertEqual(arg3, 'application/hbp-neurorobotics.sdf.world+xml')
            if (arg2 == 'folder_a/model.sdf'):
                self.assertEqual(arg3, 'application/hbp-neurorobotics.sdf.robot+xml')
            if (arg2 == 'folder_a/my_brain.py'):
                self.assertEqual(arg3, 'application/hbp-neurorobotics.brain+python')
            if (arg2 == 'folder_a/bibi_configuration.xml'):
                self.assertEqual(arg3, 'application/hbp-neurorobotics.bibi+xml')
                with open(arg1) as bibi_file:
                    bibi_dom = bibi_api_gen.CreateFromDocument(bibi_file.read())
                    self.assertEqual(
                        bibi_dom.bodyModel,
                        bibi_api_gen.SDF_Filename('model.sdf')
                    )
                    self.assertEqual(
                        bibi_dom.brainModel.file,
                        bibi_api_gen.Python_Filename('my_brain.py')
                    )
            if (arg2 == 'folder_a/experiment_configuration.xml'):
                self.assertEqual(arg3, 'application/hbp-neurorobotics+xml')
                with open(arg1) as exd_file:
                    experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())
                    self.assertEqual(experiment.environmentModel.src, 'virtual_room.sdf')
                    self.assertEqual(
                        experiment.bibiConf.src,
                        neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME
                    )

        kall = rmtree_mock.call_args
        args, kwargs = kall
        self.temporary_directory_to_clean.append(args[0])

    @patch('shutil.rmtree')
    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.get_model_basepath')
    def test_clone_experiment_template_to_collab_failure(self, get_model_basepath_mock, rmtree_mock):
        get_model_basepath_mock.return_value = os.path.join(self.models_directory)
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        mock_side_effect =\
            [
                None,
                None,
                DocException('Upload Failure')
            ]
        self.mock_document_client_instance.upload_file.side_effect = mock_side_effect
        experiment_config_path = os.path.join(self.experiments_directory, 'ExDXMLExample.exc')
        self.assertRaises(
            NRPServicesUploadException,
            neurorobotic_collab_client.clone_experiment_template_to_collab,
            'folder_a',
            experiment_config_path
        )
        self.assertEqual(
            self.mock_document_client_instance.rmdir.call_count,
            1
        )
        self.mock_document_client_instance.upload_file.side_effect = mock_side_effect
        try:
            neurorobotic_collab_client.clone_experiment_template_to_collab(
                'folder_a',
                experiment_config_path
            )
        except NRPServicesUploadException as e:
            self.assertEqual(e.message, 'Upload Failure')

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
        environment_conf = "my_world.sdf"
        robot_model = "model.sdf"
        self.mock_document_client_instance.get_path_by_id.return_value = experiment_folder
        self.mock_document_client_instance.listdir.return_value = [experiment_conf, environment_conf, robot_model]
        self.mock_document_client_instance.get_standard_attr.side_effect =\
            [
              {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics+xml','_uuid': 'a_uuid'},
              {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics.sdf.world+xml','_uuid': 'a_uuid'},
              {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics.sdf.robot+xml','_uuid': 'a_uuid'}
            ]
        result = neurorobotic_collab_client.clone_experiment_template_from_collab('some_uuid',)
        self.assertEqual(self.mock_document_client_instance.download_file_by_id.call_count, 3)
        self.assertEqual(result,
            {
                'experiment_conf': os.path.join(tmp_dir, experiment_conf),
                'environment_conf': os.path.join(tmp_dir, environment_conf),
                'robot_model': os.path.join(tmp_dir, robot_model)
            }
        )

    @patch('tempfile.mkdtemp')
    def test_clone_experiment_template_from_collab_context(self, mkdtemp_mock):
        tmp_dir = "/tmp/tmp_directory"
        mkdtemp_mock.return_value = tmp_dir
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        experiment_folder = "fake_experiment_folder"
        experiment_conf = "exdConf.xml"
        environment_conf = "model.sdf"
        robot_model = "model.sdf"
        self.mock_get_or_raise_collab_context.return_value = CollabContext(
            'context_id', os.path.dirname(experiment_conf), 'folder_uuid'
        )
        self.mock_document_client_instance.get_path_by_id.return_value = experiment_folder
        self.mock_document_client_instance.listdir.return_value = [experiment_conf, environment_conf, robot_model]
        self.mock_document_client_instance.get_standard_attr.side_effect =\
            [
                {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics+xml','_uuid': 'a_uuid'},
                {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics.sdf.world+xml','_uuid': 'a_uuid'},
                {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics.sdf.robot+xml','_uuid': 'a_uuid'}

            ]
        result = neurorobotic_collab_client.clone_experiment_template_from_collab_context()
        self.assertEqual(self.mock_document_client_instance.download_file_by_id.call_count, 3)
        print(result)
        print({
                'experiment_conf': os.path.join(tmp_dir, experiment_conf),
                'environment_conf': os.path.join(tmp_dir, environment_conf),
                'robot_model': os.path.join(tmp_dir, robot_model)
            }
        )
        self.assertEqual(result,
            {
                'experiment_conf': os.path.join(tmp_dir, experiment_conf),
                'environment_conf': os.path.join(tmp_dir, environment_conf),
                'robot_model': os.path.join(tmp_dir, robot_model)
            }
        )

    @patch('tempfile.mkdtemp')
    def test_clone_file_from_collab(self, mkdtemp_mock):
        tmp_dir = "/tmp/tmp_directory"
        mkdtemp_mock.return_value = tmp_dir
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        experiment_folder = "fake_experiment_folder"
        self.mock_document_client_instance.get_path_by_id.return_value = experiment_folder
        bibi_configuration_file = "bibi_cfg.xml"
        self.mock_document_client_instance.listdir.return_value = [
            "exdConf.xml",
            "my_world.sdf",
            "model.sdf",
            bibi_configuration_file

        ]
        self.mock_document_client_instance.get_standard_attr.side_effect =\
          [
              {'_entityType':'file', '_contentType':'application/hbp-neurorobotics+xml'},
              {'_entityType':'file', '_contentType':'application/hbp-neurorobotics+xml'},
              {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.sdf.world+xml'},
              {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.sdf.robot+xml'},
              {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.bibi+xml', '_uuid': 'a_uuid'}
          ]
        result = neurorobotic_collab_client.clone_file_from_collab('some_uuid', neurorobotic_collab_client.BIBI_CONFIGURATION_MIMETYPE, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME)
        self.assertEqual(self.mock_document_client_instance.download_file_by_id.call_count, 1)
        bibi_configuration_path = os.path.join(tmp_dir, bibi_configuration_file)
        self.assertEqual(result[0], bibi_configuration_path)
        self.assertEqual(result[1], os.path.join(experiment_folder, bibi_configuration_file))
        self.mock_document_client_instance.get_standard_attr.side_effect =\
          [
              {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.bibi+xml', '_uuid': 'a_uuid'}
          ]
        self.mock_document_client_instance.download_file_by_id.reset_mock()
        result = neurorobotic_collab_client.clone_file_from_collab('some_uuid', neurorobotic_collab_client.BIBI_CONFIGURATION_MIMETYPE, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME)
        self.assertEqual(self.mock_document_client_instance.download_file_by_id.call_count, 1)
        bibi_configuration_path = os.path.join(tmp_dir, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME)
        self.assertEqual(result[0], bibi_configuration_path)
        self.assertEqual(result[1], os.path.join(experiment_folder, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME))

    @patch(
        'hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient.clone_file_from_collab'
    )
    def test_clone_file_from_collab_context(self, mock_clone_file_from_collab):
        uuid = "some uuid"
        collab_context = MagicMock(experiment_folder_uuid=uuid)
        self.mock_get_or_raise_collab_context.return_value = collab_context
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        neurorobotic_collab_client.clone_file_from_collab_context(neurorobotic_collab_client.BIBI_CONFIGURATION_MIMETYPE, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME)
        self.assertEqual(neurorobotic_collab_client.clone_file_from_collab.call_count, 1)
        neurorobotic_collab_client.clone_file_from_collab.assert_called_with(uuid, neurorobotic_collab_client.BIBI_CONFIGURATION_MIMETYPE, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME)

    def test_clone_bibi_file_from_collab_context(self):
        # check that it calls clone_file_from_collab correctly
         client = NeuroroboticsCollabClient("token", 'aaa')
         with patch("hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient.clone_file_from_collab_context") as collab_context_mock:
             with patch("hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient._parse_and_check_file_is_valid") as check_file_is_valid_mock:
                 collab_context_mock.return_value = ("bibi file path", "bibi_remote_path")
                 check_file_is_valid_mock.return_value = "bibi obj"
                 bibi, bibi_filepath, bibi_remote_path = client.clone_bibi_file_from_collab_context()
                 self.assertEqual(bibi, "bibi obj")
                 self.assertEqual(bibi_filepath, "bibi file path")
                 self.assertEqual(collab_context_mock.call_count, 1)
                 collab_context_mock.assert_called_with(client.BIBI_CONFIGURATION_MIMETYPE, client.BIBI_CONFIGURATION_FILE_NAME)
                 self.assertEqual(check_file_is_valid_mock.call_count, 1)
                 check_file_is_valid_mock.assert_called_with("bibi file path", bibi_api_gen.CreateFromDocument, bibi_api_gen.BIBIConfiguration)
                 # check exception is raised when file could not be cloned.
                 collab_context_mock.return_value = None
                 self.assertRaises(NRPServicesGeneralException, client.clone_bibi_file_from_collab_context)

    def test_clone_exp_file_from_collab_context(self):
        # check that it calls clone_file_from_collab correctly
         client = NeuroroboticsCollabClient("token", 'aaa')
         with patch("hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient.clone_file_from_collab_context") as collab_context_mock:
             with patch("hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient._parse_and_check_file_is_valid") as check_file_is_valid_mock:
                 collab_context_mock.return_value = ("exp file path", "exp_remote_path")
                 check_file_is_valid_mock.return_value = "exp obj"
                 exp, exp_filepath, exp_remote_path = client.clone_exp_file_from_collab_context()
                 self.assertEqual(exp, "exp obj")
                 self.assertEqual(exp_filepath, "exp file path")
                 self.assertEqual(collab_context_mock.call_count, 1)
                 collab_context_mock.assert_called_with(client.EXPERIMENT_CONFIGURATION_MIMETYPE, client.EXPERIMENT_CONFIGURATION_FILE_NAME)
                 self.assertEqual(check_file_is_valid_mock.call_count, 1)
                 check_file_is_valid_mock.assert_called_with("exp file path", exp_conf_api_gen.CreateFromDocument, exp_conf_api_gen.ExD_)
                 # check exception is raised when file could not be cloned.
                 collab_context_mock.return_value = None
                 self.assertRaises(NRPServicesGeneralException, client.clone_exp_file_from_collab_context)

    def test_parse_and_check_file_is_valid(self):
        client = NeuroroboticsCollabClient("token", 'aaa')
        with patch("__builtin__.open", mock_open(read_data="file data")) as mock_file:
            # check exception is raised when the resulting object has the wrong instance type
            self.assertRaises(NRPServicesGeneralException, client._parse_and_check_file_is_valid, "filepath", str, int)
            # check an exception is raised when the xml is not parsed correctly
            self.assertRaises(NRPServicesGeneralException, client._parse_and_check_file_is_valid, "im not valid", exp_conf_api_gen.CreateFromDocument, exp_conf_api_gen.ExD_)
            # check everything works as expected with normal input
            file_content = client._parse_and_check_file_is_valid("filepath", str, str)
            self.assertEqual(file_content, "file data")

    def test_find_and_replace_file_in_collab(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        experiment_folder = "/fake_collab/fake_experiment_folder"
        experiment_conf = "exdConf.xml"
        environment_conf = "model.sdf"
        default_filename = "default_file.txt"
        fake_content = "content"
        neurorobotic_collab_client.replace_file_content_in_collab = MagicMock()
        self.mock_get_or_raise_collab_context.return_value = CollabContext(
            'context_id', os.path.dirname(experiment_conf), 'folder_uuid'
        )
        self.mock_document_client_instance.get_path_by_id.return_value = experiment_folder
        self.mock_document_client_instance.listdir.return_value = [experiment_conf, environment_conf]
        self.mock_document_client_instance.get_standard_attr.side_effect = [
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics+xml'},
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.sdf.world+xml'},
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics+xml'},
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics+xml'},
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics+xml'},
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.sdf.world+xml'}]

        neurorobotic_collab_client.find_and_replace_file_in_collab(fake_content, "fake_mimetype", default_filename)
        neurorobotic_collab_client.replace_file_content_in_collab.assert_called_with(fake_content, "fake_mimetype", os.path.join(experiment_folder, default_filename))

        self.mock_document_client_instance.get_standard_attr.side_effect = [
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics+xml'},
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.sdf.world+xml'}
        ]
        neurorobotic_collab_client.find_and_replace_file_in_collab(fake_content, "application/hbp-neurorobotics.sdf.world+xml", default_filename)
        neurorobotic_collab_client.replace_file_content_in_collab.assert_called_with(fake_content, "application/hbp-neurorobotics.sdf.world+xml", os.path.join(experiment_folder, environment_conf))

    def test_replace_file_content_in_collab(self):
        file_name = "fake_name.txt"
        dir_path ="/fake_collab/fake_experiment_folder"
        filepath = dir_path + "/"+ file_name
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        self.mock_document_client_instance.get_path_by_id.return_value = dir_path
        mimetype = "fake_mime_type"
        string_to_upload = "some_string"
        neurorobotic_collab_client.replace_file_content_in_collab(string_to_upload, mimetype, file_name)
        self.assertEqual(self.mock_document_client_instance.remove.call_count, 1)
        self.assertEqual(self.mock_document_client_instance.upload_string.call_count, 1)
        argslist = [x[0] for x in self.mock_document_client_instance.upload_string.call_args_list]
        arg1, arg2, arg3 = argslist[0]
        self.assertEqual(arg1, string_to_upload)
        self.assertEqual(arg2, filepath)
        self.assertEqual(arg3, mimetype)

    def test_add_app_to_nav_menu(self):

        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'fake-context-id')
        self.mock_collab_client_instance.get_current_tree = MagicMock(return_value={"id":"parent_id", "children": [{"app_id":"app_id", "context": "fake-context-id"}]})
        self.mock_collab_client_instance.add_item = MagicMock()

        neurorobotic_collab_client.add_app_to_nav_menu()
        self.mock_collab_client_instance.add_item.assert_called_with("parent_id", {"app_id": "app_id", "name": "New Experiment"})

    def test_get_mimetype(self):
        ncc = NeuroroboticsCollabClient("token", 'aaa')
        self.assertIsNotNone(ncc.get_mimetype(ncc.EXPERIMENT_CONFIGURATION_FILE_NAME))
        self.assertIsNotNone(ncc.get_mimetype(ncc.BIBI_CONFIGURATION_FILE_NAME))
        world_file_path = os.path.join(self.models_directory, 'virtual_room/virtual_room.sdf')
        self.assertEqual(ncc.get_mimetype(world_file_path), ncc.SDF_WORLD_MIMETYPE)
        image_file_path = os.path.join(self.experiments_directory, 'ExDXMLExample.png')
        self.assertEqual(ncc.get_mimetype(image_file_path), ncc.PNG_IMAGE_MIMETYPE)
        model_file_path = os.path.join(self.models_directory, 'husky_model/model.sdf')
        self.assertEqual(ncc.get_mimetype(model_file_path), ncc.SDF_ROBOT_MIMETYPE)
        brain_file_path = os.path.join(self.models_directory, 'my_brain.py')
        self.assertEqual(
            ncc.get_mimetype(brain_file_path),
            ncc.BRAIN_PYNN_MIMETYPE
        )
        brain_file_path = os.path.join(self.models_directory, 'my_brain_2.py')
        self.assertEqual(
            ncc.get_mimetype(brain_file_path),
            ncc.BRAIN_PYNN_MIMETYPE
        )
        brain_file_path = os.path.join(self.models_directory, 'my_brain_3.py')
        self.assertEqual(
            ncc.get_mimetype(brain_file_path),
            ncc.BRAIN_PYNN_MIMETYPE
        )
        brain_file_path = os.path.join(self.models_directory, 'my_brain_4.py')
        self.assertEqual(
            ncc.get_mimetype(brain_file_path),
            ncc.BRAIN_PYNN_MIMETYPE
        )
    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.get_model_basepath')
    def test_flatten_bibi_configuration(self, get_model_basepath_mock):
        get_model_basepath_mock.return_value = self.models_directory
        exp_configuration = os.path.join(self.experiments_directory, 'ExDXMLExample.exc')
        expected_list = [
            NeuroroboticsCollabClient.EXPERIMENT_CONFIGURATION_FILE_NAME,
            'braitenberg_husky_linear_twist.py',
            'braitenberg_husky_non_linear_twist.py',
            'virtual_room.sdf',
            'model.sdf',
            'ExDXMLExample.png',
            NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME,
            'my_brain.py',
            'test_sm.py'
        ]

        with _FlattenedExperimentDirectory(exp_configuration) as temporary_folder:
            bibi_configuration_file = os.path.join(self.experiments_directory, 'milestone2_python_tf.bibi')
            l = os.listdir(temporary_folder)
            # make sure we do not copy things we aren't expecting
            self.assertEqual(len(l), len(expected_list))
            for expected_file in expected_list:
                self.assertIn(expected_file, l)
            flattened_bibi_configuration_file = os.path.join(
                temporary_folder,
                NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME
            )
            flattened_exp_configuration_file = os.path.join(
                temporary_folder,
                NeuroroboticsCollabClient.EXPERIMENT_CONFIGURATION_FILE_NAME
            )
            with open(flattened_exp_configuration_file) as e:
                exp_configuration_dom = exp_conf_api_gen.CreateFromDocument(e.read())

            for sm in exp_configuration_dom.experimentControl.stateMachine:
                self.assertEqual(sm.src, os.path.basename(sm.src))
                path = os.path.join(temporary_folder, sm.src)
                self.assertEqual(os.path.exists(path), True)

            with open(flattened_bibi_configuration_file) as b:
                bibi_configuration_dom = bibi_api_gen.CreateFromDocument(b.read())

            for tf in bibi_configuration_dom.transferFunction:
                if hasattr(tf, "src") and tf.src:
                    self.assertEqual(tf.src, os.path.basename(tf.src))
                    path = os.path.join(temporary_folder, tf.src)
                    self.assertEqual(os.path.exists(path), True)

            self.assertEqual(
                bibi_configuration_dom.bodyModel,
                bibi_api_gen.SDF_Filename('model.sdf')
            )
            self.assertEqual(
                bibi_configuration_dom.brainModel.file,
                bibi_api_gen.Python_Filename('my_brain.py')
            )
    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.get_model_basepath')
    def test_flatten_bibi_configuration_retina(self, get_model_basepath_mock):
        get_model_basepath_mock.return_value = self.models_directory
        exp_configuration = os.path.join(self.experiments_directory, 'ExDXMLExampleRetina.exc')
        expected_list = [
            NeuroroboticsCollabClient.EXPERIMENT_CONFIGURATION_FILE_NAME,
            'grab_image_retina_1.py',
            'retina_config_1.py',
            'retina_config_2.py',
            'virtual_room.sdf',
            'model.sdf',
            'ExDXMLExample.png',
            NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME,
            'my_brain.py'
        ]

        with _FlattenedExperimentDirectory(exp_configuration) as temporary_folder:
            bibi_configuration_file = os.path.join(self.models_directory, 'BIBI/retina_bibi.xml')
            l = os.listdir(temporary_folder)
            # make sure we do not copy things we aren't expecting
            self.assertEqual(len(l), len(expected_list))
            for expected_file in expected_list:
                self.assertIn(expected_file, l)
            flattened_bibi_configuration_file = os.path.join(
                temporary_folder,
                NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME
            )
            with open(flattened_bibi_configuration_file) as b:
                bibi_configuration_dom = bibi_api_gen.CreateFromDocument(b.read())

            for tf in bibi_configuration_dom.transferFunction:
                if hasattr(tf, "src") and tf.src:
                    self.assertEqual(tf.src, os.path.basename(tf.src))
                    path = os.path.join(temporary_folder, tf.src)
                    self.assertEqual(os.path.exists(path), True)

            self.assertEqual(
                bibi_configuration_dom.bodyModel,
                bibi_api_gen.SDF_Filename('model.sdf')
            )
            self.assertEqual(
                bibi_configuration_dom.brainModel.file,
                bibi_api_gen.Python_Filename('my_brain.py')
            )

    def test_populate_subfolder_in_collab(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        class FileObject(object):
            def __init__(self, name, temporary_path):
                self.name = name
                self.temporary_path = temporary_path

        files = [
          FileObject('left_wheel_join_position.csv', '/tmp/csv_recorders/left_wheel_join_position.csv'),
          FileObject('right_wheel_join_position.csv', '/tmp/csv_recorders/right_wheel_join_position.csv')
        ]
        subfoldername = 'csv_recorders_2016-11-23_02_54_01'
        mimetype = 'text/csv'
        neurorobotic_collab_client.populate_subfolder_in_collab(
          subfoldername,
          files,
          mimetype
        )
        self.mock_document_client_instance.mkdir.assert_called_with(subfoldername)
        argslist = [x[0] for x in self.mock_document_client_instance.upload_file.call_args_list]
        self.assertIn((files[0].temporary_path, subfoldername + '/' + files[0].name, mimetype), argslist)
        self.assertIn((files[1].temporary_path, subfoldername + '/' + files[1].name, mimetype), argslist)
        self.assertEqual(self.mock_document_client_instance.rmdir.call_count, 1)
        neurorobotic_collab_client.populate_subfolder_in_collab(
          subfoldername,
          files,
          mimetype
        )
        self.mock_document_client_instance.rmdir.assert_called_with(subfoldername)

        self.mock_document_client_instance.upload_file.side_effect = [None, DocException('Upload Failure')]
        self.assertRaises(
            NRPServicesUploadException,
            neurorobotic_collab_client.populate_subfolder_in_collab,
            subfoldername,
            files,
           'text/csv'
        )


if __name__ == '__main__':
    unittest.main()
