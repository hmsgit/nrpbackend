# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
NeuroroboticsCollabClient unit test
"""

import inspect
import unittest
import shutil
import os
from mock import patch, MagicMock, mock_open, call
from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
    import NeuroroboticsCollabClient, _FlattenedExperimentDirectory
from hbp_nrp_backend.collab_interface import NRPServicesUploadException
from hbp_service_client.document_service.exceptions import DocException
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen, model_conf_api_gen
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
            'hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.Client'
        )
        self.addCleanup(document_client_patcher.stop)
        self.mock_document_client = document_client_patcher.start()
        storageMock = MagicMock()
        self.mock_document_client.new = MagicMock(return_value=storageMock)
        self.mock_document_client_instance = self.mock_document_client.new().storage

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
        self.mock_document_client_instance.list_project_content.return_value = {'results':[{'name':'a'}, {'name':'a_0'}, {'name':'b_0'}], 'next': None}
        self.assertEqual(neurorobotic_collab_client.generate_unique_folder_name('a'), 'a_1')
        self.assertEqual(neurorobotic_collab_client.generate_unique_folder_name('b'), 'b')
        # Test the paging
        self.mock_document_client_instance.list_project_content.side_effect = [{'results':[{'name':'a'}, {'name':'a_0'}, {'name':'b_0'}], 'next': 'get_more_pages!'},
                                                                               {'results':[{'name':'a_1'}, {'name':'a_2'}, {'name':'a_3'}], 'next': None}]
        self.assertEqual(neurorobotic_collab_client.generate_unique_folder_name('a'), 'a_4')

    @patch('shutil.rmtree')
    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.get_model_basepath')
    def test_clone_experiment_template_to_collab_success(self, get_model_basepath_mock, rmtree_mock):
        get_model_basepath_mock.return_value = self.models_directory
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')

        neurorobotic_collab_client._get_project_uuid = MagicMock(return_value="parent_uuid")
        self.mock_document_client_instance.create_folder.return_value = {"uuid": "folder_uuid"}
        self.mock_document_client_instance.create_file.return_value = {"uuid": "created_file_uuid"}

        neurorobotic_collab_client.clone_experiment_template_to_collab(
            'folder_a',
            os.path.join(self.experiments_directory, 'ExDXMLExample.exc')
        )

        self.mock_document_client_instance.create_folder.assert_called_with('folder_a', parent="parent_uuid")

        call_list = [
            call('experiment_configuration.exc', content_type='application/hbp-neurorobotics+xml', parent='folder_uuid'),
            call('ExDXMLExample.png', content_type='image/png', parent='folder_uuid'),
            call('virtual_room.sdf', content_type='application/hbp-neurorobotics.sdf.world+xml', parent='folder_uuid'),
            call('model.sdf', content_type='application/hbp-neurorobotics.sdf.robot+xml', parent='folder_uuid'),
            call('bibi_configuration.bibi', content_type='application/hbp-neurorobotics.bibi+xml', parent='folder_uuid'),
            call('braitenberg_husky_non_linear_twist.py', content_type='application/hbp-neurorobotics.tfs+python', parent='folder_uuid'),
            call('braitenberg_husky_linear_twist.py', content_type='application/hbp-neurorobotics.tfs+python', parent='folder_uuid'),
            call('my_brain.py', content_type='application/hbp-neurorobotics.brain+python', parent='folder_uuid'),
        ]
        self.mock_document_client_instance.create_file.assert_has_calls(call_list, any_order=True)

        # get the arguments for the file_upload and check they are correct
        # and that the place the file is being uploaded from exists
        for each_call in self.mock_document_client_instance.upload_file_content.call_args_list:
            args, kwargs = each_call
            file_uuid = args[0]
            each_file = kwargs['source']

            self.assertEqual(file_uuid, 'created_file_uuid')
            self.assertTrue(os.path.exists(each_file))
            if each_file.endswith('.bibi'):
                with open(each_file) as bibi_file:
                    bibi_dom = bibi_api_gen.CreateFromDocument(bibi_file.read())
                    self.assertEqual(
                        bibi_dom.bodyModel,
                        bibi_api_gen.SDF_Filename('model.sdf')
                    )
                    self.assertEqual(
                        bibi_dom.brainModel.file,
                        bibi_api_gen.Python_Filename('my_brain.py')
                    )

            elif each_file.endswith('.exc'):
                with open(each_file) as exd_file:
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

        neurorobotic_collab_client._get_project_uuid = MagicMock(return_value="parent_uuid")
        self.mock_document_client_instance.create_folder.return_value = {"uuid": "folder_uuid"}
        self.mock_document_client_instance.create_file.return_value = {"uuid": "created_file_uuid"}
        experiment_config_path = os.path.join(self.experiments_directory, 'ExDXMLExample.exc')
        self.mock_document_client_instance.upload_file_content.side_effect = [None, None, DocException('Upload Failure')]

        with self.assertRaises(NRPServicesUploadException) as exc:
            neurorobotic_collab_client.clone_experiment_template_to_collab(
                'folder_a',
                experiment_config_path,
                None
            )

        self.assertEqual(exc.exception.message, 'Upload Failure')
        self.assertTrue(self.mock_document_client_instance.delete_folder.called)

        kall = rmtree_mock.call_args
        args, kwargs = kall
        self.temporary_directory_to_clean.append(args[0])

    @patch('tempfile.mkdtemp')
    def test_clone_experiment_template_from_collab(self, mkdtemp_mock):
        tmp_dir = "/tmp/tmp_directory"
        mkdtemp_mock.return_value = tmp_dir
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        experiment_conf = "exdConf.xml"
        environment_conf = "my_world.sdf"
        robot_model = "model.sdf"
        neurorobotic_collab_client.find_file_in_collab = MagicMock(return_value={'results': [{'name': experiment_conf, 'content_type': 'application/hbp-neurorobotics+xml','uuid': 'a_uuid'},
                                                                                           {'name': environment_conf, 'content_type': 'application/hbp-neurorobotics.sdf.world+xml','uuid': 'b_uuid'},
                                                                                           {'name': robot_model, 'content_type': 'application/hbp-neurorobotics.sdf.robot+xml','uuid': 'c_uuid'}]}['results'])
        self.mock_document_client_instance.download_file_content.side_effect = [["some_content1"], ["some_content2"], ["some_content3"]]
        m = mock_open()
        with patch("__builtin__.open", m):
            result = neurorobotic_collab_client.clone_experiment_template_from_collab()

        self.mock_document_client_instance.download_file_content.assert_has_calls([call('a_uuid'),
                                                                                   call('b_uuid'),
                                                                                   call('c_uuid')],
                                                                                  any_order=True)
        m.assert_has_calls([call(os.path.join(tmp_dir,experiment_conf), 'w'),
                            call(os.path.join(tmp_dir, environment_conf), 'w'),
                            call(os.path.join(tmp_dir, robot_model), 'w')],
                           any_order=True)
        handle = m()
        handle.write.assert_has_calls([call('some_content1'),
                                 call('some_content2'),
                                 call('some_content3')])
        neurorobotic_collab_client.find_file_in_collab.assert_called_with()
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
        experiment_conf = "exdConf.xml"
        mimetype = 'application/hbp-neurorobotics+xml'
        neurorobotic_collab_client.find_file_in_collab = MagicMock(return_value={'name': experiment_conf, 'content_type': 'application/hbp-neurorobotics+xml','uuid': 'a_uuid'})

        self.mock_document_client_instance.download_file_content.return_value = ["some_content1"]
        m = mock_open()
        with patch("__builtin__.open", m):
            saved_localpath, saved_file_uuid = neurorobotic_collab_client.clone_file_from_collab(mimetype)
            self.assertEqual(saved_file_uuid, 'a_uuid')
            self.assertEqual(saved_localpath, os.path.join(tmp_dir, experiment_conf))

        self.mock_document_client_instance.download_file_content.assert_called_with('a_uuid')
        m.assert_called_with(os.path.join(tmp_dir,experiment_conf), 'w')

        handle = m()
        handle.write.assert_called_with('some_content1')

        neurorobotic_collab_client.find_file_in_collab.assert_called_with(mimetype)

    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient._parse_and_check_file_is_valid')
    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient.clone_file_from_collab')
    def test_clone_bibi_file_from_collab_context(self, clone_file_mock, check_file_is_valid_mock):
         client = NeuroroboticsCollabClient("token", 'aaa')
         clone_file_mock.return_value = ("bibi file path", "bibi_remote_uuid")
         check_file_is_valid_mock.return_value = "bibi obj"
         bibi, bibi_filepath, bibi_remote_path = client.clone_bibi_file_from_collab_context()

         self.assertEqual(bibi, "bibi obj")
         self.assertEqual(bibi_filepath, "bibi file path")
         self.assertEqual(clone_file_mock.call_count, 1)

         clone_file_mock.assert_called_with(client.BIBI_CONFIGURATION_MIMETYPE)
         self.assertEqual(check_file_is_valid_mock.call_count, 1)
         check_file_is_valid_mock.assert_called_with("bibi file path", bibi_api_gen.CreateFromDocument, bibi_api_gen.BIBIConfiguration)

         # check exception is raised when file could not be cloned.
         clone_file_mock.return_value = None
         self.assertRaises(NRPServicesGeneralException, client.clone_bibi_file_from_collab_context)

    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient._parse_and_check_file_is_valid')
    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.NeuroroboticsCollabClient.clone_file_from_collab')
    def test_clone_exp_file_from_collab_context(self, clone_file_mock, check_file_is_valid_mock):
         client = NeuroroboticsCollabClient("token", 'aaa')
         #client.
         clone_file_mock.return_value = ("exp file path", "exp_remote_uuid")
         check_file_is_valid_mock.return_value = "exp obj"
         exp, exp_filepath, exp_remote_path = client.clone_exp_file_from_collab_context()

         self.assertEqual(exp, "exp obj")
         self.assertEqual(exp_filepath, "exp file path")
         self.assertEqual(clone_file_mock.call_count, 1)

         clone_file_mock.assert_called_with(client.EXPERIMENT_CONFIGURATION_MIMETYPE)
         self.assertEqual(check_file_is_valid_mock.call_count, 1)
         check_file_is_valid_mock.assert_called_with("exp file path", exp_conf_api_gen.CreateFromDocument, exp_conf_api_gen.ExD_)

         # check exception is raised when file could not be cloned.
         clone_file_mock.return_value = None
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

    def test_replace_file_content_collab(self):
        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')
        neurorobotic_collab_client.find_file_in_collab = MagicMock()
        # Check when uuid is given
        neurorobotic_collab_client.replace_file_content_in_collab("my_content", "my_uuid")

        self.mock_document_client_instance.upload_file_content.assert_called_with("my_uuid", content="my_content")
        self.assertEqual(neurorobotic_collab_client.find_file_in_collab.called, False)
        self.assertEqual(self.mock_document_client_instance.create_file.called, False)
        # check when uuid is not given but mimetype is but no filename
        neurorobotic_collab_client.find_file_in_collab = MagicMock(return_value={'uuid':'my_new_uuid'})
        neurorobotic_collab_client.replace_file_content_in_collab("my_content", mimetype="my_mimetype")

        self.mock_document_client_instance.upload_file_content.assert_called_with("my_new_uuid", content="my_content")
        neurorobotic_collab_client.find_file_in_collab.assert_called_with("my_mimetype", None)
        self.assertEqual(self.mock_document_client_instance.create_file.called, False)
        # check when uuid is not given but mimetype and filename is
        self.mock_document_client_instance.create_file = MagicMock(return_value={"uuid": "created_uuid"})
        neurorobotic_collab_client.find_file_in_collab = MagicMock(return_value=None)
        neurorobotic_collab_client.replace_file_content_in_collab("my_content", mimetype="my_other_mimetype", filename="my_filename")

        self.mock_document_client_instance.upload_file_content.assert_called_with("created_uuid", content="my_content")
        neurorobotic_collab_client.find_file_in_collab.assert_called_with("my_other_mimetype", "my_filename")
        self.mock_document_client_instance.create_file.assert_called_with("my_filename", "my_other_mimetype", self.mock_get_or_raise_collab_context().experiment_folder_uuid)


    def test_find_file_in_collab(self):

        neurorobotic_collab_client = NeuroroboticsCollabClient("token", 'aaa')

        neurorobotic_collab_client.experiment_folder_content = False
        self.mock_document_client_instance.list_folder_content = MagicMock(return_value={'results': 'fake_result'})
        # Check when we give no parameters (we want to get everything)
        self.assertEqual(neurorobotic_collab_client.find_file_in_collab(), 'fake_result')
        self.mock_document_client_instance.list_folder_content.assert_called_with(self.mock_get_or_raise_collab_context().experiment_folder_uuid, entity_type='file')
        # Check when we dont request again when we already got the data before
        self.mock_document_client_instance.list_folder_content.reset_mock()
        self.assertEqual(neurorobotic_collab_client.find_file_in_collab(), 'fake_result')
        self.assertEqual(self.mock_document_client_instance.list_folder_content.called, False)
        # Check when we give a mimetype and no filename
        neurorobotic_collab_client.experiment_folder_content = [{'name': 'name_a', 'content_type': 'content_a'},
                                                                {'name': 'name_b', 'content_type': 'content_b'},
                                                                {'name': 'name_c', 'content_type': 'content_c'},
                                                                {'name': 'name_d', 'content_type': 'content_a'},
                                                                {'name': 'name_d', 'content_type': 'content_d'}
                                                                ]
        self.assertEqual(neurorobotic_collab_client.find_file_in_collab('content_a'), {'name': 'name_a', 'content_type': 'content_a'})
        # Check when we give a filename and no mimetype
        self.assertEqual(neurorobotic_collab_client.find_file_in_collab(filename='name_b'), {'name': 'name_b', 'content_type': 'content_b'})
        # Check when we gve a mimetype and a filename
        self.assertEqual(neurorobotic_collab_client.find_file_in_collab('content_a', 'name_d'), {'name': 'name_d', 'content_type': 'content_a'})
        self.assertEqual(neurorobotic_collab_client.find_file_in_collab('content_d', 'name_d'), {'name': 'name_d', 'content_type': 'content_d'})

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

        with _FlattenedExperimentDirectory(exp_configuration, None) as temporary_folder:
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

        with _FlattenedExperimentDirectory(exp_configuration, None) as temporary_folder:
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
        neurorobotic_collab_client._get_path_by_id = MagicMock(return_value='exp_path')
        self.mock_document_client_instance.get_entity_by_query.return_value = {"uuid": "folder_uuid"}
        self.mock_document_client_instance.create_folder.return_value = {"uuid": "new_folder_uuid"}
        self.mock_document_client_instance.create_file.side_effect = [{"uuid": "uuid_1"}, {"uuid": "uuid_2"}]

        neurorobotic_collab_client.populate_subfolder_in_collab(subfoldername, files, mimetype)

        self.mock_document_client_instance.create_folder.assert_called_with(subfoldername, parent=self.mock_get_or_raise_collab_context().experiment_folder_uuid)

        create_call_list = [call(files[0].name, content_type=mimetype, parent="new_folder_uuid"), call(files[1].name, content_type=mimetype, parent="new_folder_uuid")]
        self.mock_document_client_instance.create_file.assert_has_calls(create_call_list)
        upload_call_list = [call("uuid_1", source=files[0].temporary_path), call("uuid_2", source=files[1].temporary_path)]
        self.mock_document_client_instance.upload_file_content.assert_has_calls(upload_call_list)

        self.assertEqual(self.mock_document_client_instance.delete_folder.call_count, 1)

        self.mock_document_client_instance.get_entity_by_query.assert_called_with(path=os.path.join("exp_path", subfoldername))
        self.mock_document_client_instance.delete_folder.assert_called_with("folder_uuid")
        self.mock_document_client_instance.create_file.side_effect = [{"uuid": "uuid_1"}, {"uuid": "uuid_2"}]
        self.mock_document_client_instance.upload_file_content.side_effect = [None, DocException('Upload Failure')]
        self.assertRaises(
            NRPServicesUploadException,
            neurorobotic_collab_client.populate_subfolder_in_collab,
            subfoldername,
            files,
           'text/csv'
        )

    @patch('hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient.get_model_basepath')
    def test_flattened_parse_model_config_files(self, get_model_basepath_mock):
        expected_list = ['fakeRobot.sdf',
                         'experiment_configuration.exc',
                         'TemplateEmpty.png',
                         'TemplateEmpty.json',
                         'bibi_configuration.bibi',
                         'fakeEnv.sdf',
                         'fakeBrain.py']
        get_model_basepath_mock.return_value = self.models_directory
        exp_configuration = os.path.join(self.experiments_directory, 'experiment_configuration.exc')
        mock_paths = {
            "envPath": os.path.join("fakeEnvFolder", "model.config"),
            "robotPath": os.path.join("fakeRobotFolder", "model.config"),
            "brainPath": os.path.join("fakeBrainFolder", "fakeBrain.py")
        }
        with _FlattenedExperimentDirectory(exp_configuration, mock_paths) as temporary_folder:
            self.assertEqual('fakeRobot.sdf' in os.listdir(temporary_folder), True)
            self.assertEqual('experiment_configuration.exc' in os.listdir(temporary_folder), True)
            self.assertEqual('TemplateEmpty.png' in os.listdir(temporary_folder), True)
            self.assertEqual('TemplateEmpty.json' in os.listdir(temporary_folder), True)
            self.assertEqual('bibi_configuration.bibi' in os.listdir(temporary_folder), True)
            self.assertEqual('fakeBrain.py' in os.listdir(temporary_folder), True)
if __name__ == '__main__':
    unittest.main()
