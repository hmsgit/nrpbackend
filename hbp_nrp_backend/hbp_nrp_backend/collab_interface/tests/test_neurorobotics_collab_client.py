"""
NeuroroboticsCollabClient unit test
"""

import inspect
import unittest
import shutil
import os
from mock import patch, MagicMock
from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
    import NeuroroboticsCollabClient, _FlattenedExperimentDirectory
from hbp_nrp_backend.collab_interface import NRPServicesUploadException
from bbp_client.document_service.exceptions import DocException
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from hbp_nrp_backend.rest_server.__CollabContext import CollabContext
from hbp_nrp_backend.rest_server import app


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
    def test_clone_experiment_template_to_collab_success(self, rmtree_mock):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        neurorobotic_collab_client.clone_experiment_template_to_collab(
            'folder_a',
            os.path.join(self.models_directory, 'ExDConf', 'ExDXMLExample.xml')
        )
        self.mock_document_client_instance.mkdir.assert_called_with('folder_a')
        argslist = [x[0] for x in self.mock_document_client_instance.upload_file.call_args_list]
        self.assertTrue(any('folder_a/experiment_configuration.xml' in args for args in argslist))
        self.assertTrue(any('folder_a/ExDXMLExample.png' in args for args in argslist))
        self.assertTrue(any('folder_a/virtual_room.sdf' in args for args in argslist))
        self.assertTrue(any('folder_a/model.sdf' in args for args in argslist))
        self.assertTrue(any('folder_a/bibi_configuration.xml' in args for args in argslist))
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
    def test_clone_experiment_template_to_collab_failure(self, rmtree_mock):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        mock_side_effect =\
            [
                None,
                None,
                DocException('Upload Failure')
            ]
        self.mock_document_client_instance.upload_file.side_effect = mock_side_effect
        #self.mock_document_client_instance.rmdir = MagicMock()
        experiment_config_path = os.path.join(self.models_directory, 'ExDConf', 'ExDXMLExample.xml')
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
              {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics+xml'},
              {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics.sdf.world+xml'},
              {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics.sdf.robot+xml'}
            ]
        result = neurorobotic_collab_client.clone_experiment_template_from_collab('some_uuid',)
        self.assertEqual(self.mock_document_client_instance.download_file.call_count, 3)
        self.assertEqual(result,
            {
                'experiment_conf': os.path.join(tmp_dir, experiment_conf),
                'environment_conf': os.path.join(tmp_dir, environment_conf),
                'robot_model': os.path.join(tmp_dir, robot_model)
            }
        )

    @patch('tempfile.mkdtemp')
    def test_write_file_with_content_in_collab(self, mkdtemp_mock):
        tmp_dir = "/tmp/tmp_directory"
        mkdtemp_mock.return_value = tmp_dir
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        self.mock_document_client_instance.get_path_by_id.return_value = "fake_experiment_folder"
        self.mock_document_client_instance.remove = MagicMock()
        self.mock_document_client_instance.upload_string = MagicMock()
        result = neurorobotic_collab_client.write_file_with_content_in_collab(
            'some content', neurorobotic_collab_client.SDF_WORLD_MIMETYPE, 'world_file.sdf'
        )
        self.assertEqual(self.mock_document_client_instance.remove.call_count, 1)
        self.assertEqual(self.mock_document_client_instance.upload_string.call_count, 1)
        self.assertEqual(self.mock_document_client_instance.upload_string.call_args[0],
            ('some content', 'fake_experiment_folder/world_file.sdf', neurorobotic_collab_client.SDF_WORLD_MIMETYPE)
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
                {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics+xml'},
                {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics.sdf.world+xml'},
                {'_entityType': 'file', '_contentType': 'application/hbp-neurorobotics.sdf.robot+xml'}

            ]
        result = neurorobotic_collab_client.clone_experiment_template_from_collab_context()
        self.assertEqual(self.mock_document_client_instance.download_file.call_count, 3)
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
              {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.bibi+xml'}
          ]
        result = neurorobotic_collab_client.clone_file_from_collab('some_uuid', neurorobotic_collab_client.BIBI_CONFIGURATION_MIMETYPE, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME)
        self.assertEqual(self.mock_document_client_instance.download_file.call_count, 1)
        bibi_configuration_path = os.path.join(tmp_dir, bibi_configuration_file)
        self.assertEqual(result, bibi_configuration_path)
        self.mock_document_client_instance.get_standard_attr.side_effect =\
          [
              {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.bibi+xml'}
          ]
        self.mock_document_client_instance.download_file.reset_mock()
        result = neurorobotic_collab_client.clone_file_from_collab('some_uuid', neurorobotic_collab_client.BIBI_CONFIGURATION_MIMETYPE, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME)
        self.assertEqual(self.mock_document_client_instance.download_file.call_count, 1)
        bibi_configuration_path = os.path.join(tmp_dir, neurorobotic_collab_client.BIBI_CONFIGURATION_FILE_NAME)
        self.assertEqual(result, bibi_configuration_path)

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

    def test_get_first_file_path_with_mimetype(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        experiment_folder = "/fake_collab/fake_experiment_folder"
        experiment_conf = "exdConf.xml"
        environment_conf = "model.sdf"
        default_filename = "default_file.txt"
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

        result = neurorobotic_collab_client.get_first_file_path_with_mimetype("fake_mimetype", "fake_potential_name.txt", "default_file.txt")
        self.assertEqual(result, os.path.join(experiment_folder,default_filename))
        result = neurorobotic_collab_client.get_first_file_path_with_mimetype("application/hbp-neurorobotics.sdf.world+xml", "fake_potential_name.txt", "default_file.txt")
        self.assertEqual(result, os.path.join(experiment_folder,environment_conf))
        self.mock_document_client_instance.get_standard_attr.side_effect = [
            {'_entityType':'file', '_contentType':'application/hbp-neurorobotics.sdf.world+xml'}
        ]
        result = neurorobotic_collab_client.get_first_file_path_with_mimetype("application/hbp-neurorobotics.sdf.world+xml", "fake_potential_name.txt", "default_file.txt")
        self.assertEqual(result, os.path.join(experiment_folder,"fake_potential_name.txt"))

    def test_replace_file_content_in_collab(self):
        filepath = "/fake_collab/fake_experiment_folder/file.txt"
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        neurorobotic_collab_client.get_first_file_path_with_mimetype = MagicMock()
        neurorobotic_collab_client.get_first_file_path_with_mimetype.return_value = filepath
        mimetype = "fake_mime_type"
        string_to_upload = "some_string"
        neurorobotic_collab_client.replace_file_content_in_collab(string_to_upload, mimetype, "fake_name.txt", "default_file_name.txt")
        self.assertEqual(self.mock_document_client_instance.remove.call_count, 1)
        self.assertEqual(self.mock_document_client_instance.upload_string.call_count, 1)
        argslist = [x[0] for x in self.mock_document_client_instance.upload_string.call_args_list]
        arg1, arg2, arg3 = argslist[0]
        self.assertEqual(arg1, string_to_upload)
        self.assertEqual(arg2, filepath)
        self.assertEqual(arg3, mimetype)

    def test_download_file_from_collab(self):

        fake_filepath = '/abc/def'
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')

        neurorobotic_collab_client.download_file_from_collab(fake_filepath)

        self.mock_document_client_instance.download_file.assert_called_with(fake_filepath, None)

    def test_get_mimetype(self):
        ncc = NeuroroboticsCollabClient("token", 'aaa')
        self.assertIsNotNone(ncc.get_mimetype(ncc.EXPERIMENT_CONFIGURATION_FILE_NAME))
        self.assertIsNotNone(ncc.get_mimetype(ncc.BIBI_CONFIGURATION_FILE_NAME))
        world_file_path = os.path.join(self.models_directory, 'virtual_room/virtual_room.sdf')
        self.assertEqual(ncc.get_mimetype(world_file_path), ncc.SDF_WORLD_MIMETYPE)
        image_file_path = os.path.join(self.models_directory, 'ExDConf/ExDXMLExample.png')
        self.assertEqual(ncc.get_mimetype(image_file_path), ncc.PNG_IMAGE_MIMETYPE)
        model_file_path = os.path.join(self.models_directory, 'husky_model/model.sdf')
        self.assertEqual(ncc.get_mimetype(model_file_path), ncc.SDF_ROBOT_MIMETYPE)
        brain_file_path = os.path.join(self.models_directory, 'brain_model/my_brain.py')
        self.assertEqual(
            ncc.get_mimetype(brain_file_path),
            ncc.BRAIN_PYNN_MIMETYPE
        )
        brain_file_path = os.path.join(self.models_directory, 'brain_model/my_brain_2.py')
        self.assertEqual(
            ncc.get_mimetype(brain_file_path),
            ncc.BRAIN_PYNN_MIMETYPE
        )

    def test_flatten_bibi_configuration(self):
        exp_configuration = os.path.join(self.models_directory, 'ExDConf', 'ExDXMLExample.xml')
        expected_list = [
            NeuroroboticsCollabClient.EXPERIMENT_CONFIGURATION_FILE_NAME,
            'braitenberg_husky_linear_twist.py',
            'braitenberg_husky_non_linear_twist.py',
            'virtual_room.sdf',
            'model.sdf',
            'ExDXMLExample.png',
            NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME,
            'my_brain.py'
        ]

        with _FlattenedExperimentDirectory(exp_configuration) as temporary_folder:
            bibi_configuration_file = os.path.join(self.models_directory, 'BIBI/milestone2_python_tf.xml')
            l = os.listdir(temporary_folder)
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
