# # ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# # This file is part of the Neurorobotics Platform software
# # Copyright (C) 2014,2015,2016,2017 Human Brain Project
# # https://www.humanbrainproject.eu
# #
# # The Human Brain Project is a European Commission funded project
# # in the frame of the Horizon2020 FET Flagship plan.
# # http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
# #
# # This program is free software; you can redistribute it and/or
# # modify it under the terms of the GNU General Public License
# # as published by the Free Software Foundation; either version 2
# # of the License, or (at your option) any later version.
# #
# # This program is distributed in the hope that it will be useful,
# # but WITHOUT ANY WARRANTY; without even the implied warranty of
# # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# # GNU General Public License for more details.
# #
# # You should have received a copy of the GNU General Public License
# # along with this program; if not, write to the Free Software
# # Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# # ---LICENSE-END
# """
# Unit tests for the sim config model
# """
#
# import unittest
# import os
# from mock import patch, Mock, MagicMock, ANY, mock_open
# from hbp_nrp_commons.MockUtil import MockUtil
# from hbp_nrp_cle.robotsim.RobotManager import Robot
#
# from hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly import CLEGazeboSimulationAssembly
#
# __author__ = 'Hossain Mahmud'
#
# _base_path = 'hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.'
#
#
# @patch(_base_path + 'logging', new=MagicMock())
# @patch(_base_path + 'brainconfig', new=MagicMock())
# class TestCLEGazeboSimulationAssembly(unittest.TestCase):
#     def setUp(self):
#         # mock os
#         self.m_os = MockUtil.fakeit(self, _base_path + 'os')
#         # restore os.path.join and os.path.dirname
#         self.m_os.path.join = os.path.join
#         self.m_os.path.dirname = os.path.dirname
#
#         self.m_subproc = MockUtil.fakeit(self, _base_path + 'subprocess')
#         self.m_res_compile = MockUtil.fakeit(self, _base_path + 'compile_restricted')
#         self.m_storage = MockUtil.fakeit(self, _base_path + 'StorageClient')
#         self.m_settings = MockUtil.fakeit(self, _base_path + 'Settings')
#         self.m_settings.nrp_models_directory = '/model/base/path/'
#         self.m_simUtil = MockUtil.fakeit(self, _base_path + 'SimUtil')
#         self.m_simUtil.find_file_in_paths.return_value = '/model/base/path/xyz.model'
#
#         self.m_ziputil = MockUtil.fakeit(self, _base_path + 'ZipUtil')
#         self.m_server = MockUtil.fakeit(self, _base_path + 'ROSCLEServer')
#         self.m_dcle = MockUtil.fakeit(self, _base_path + 'DeterministicClosedLoopEngine')
#         self.m_cle = MockUtil.fakeit(self, _base_path + 'ClosedLoopEngine')
#         self.m_nrp = MockUtil.fakeit(self, _base_path + 'nrp')
#
#         self.m_simconf = MagicMock()
#         self.m_simconf.sim_id = 123
#         self.m_simconf.timeout = 'YYYY-MM-DD-HH:MM:SS'
#         self.m_simconf.timeout_type = 'real'
#         self.m_simconf.gzserver = 'real'
#         self.m_simconf.ros_notificator = 'real'
#         self.m_simconf.world_model.resource_path.abs_path = '/my/experiment/world.sdf'
#         self.m_simconf.robot_models = {
#             'bb8': Robot('bb8', '/my/experiment/bb8/model.sdf', 'my bb8', Mock()),
#             'r2d2': Robot('r2d2', '/my/experiment/bb8/model.sdf', 'my r2d2', Mock(),
#                           True, '/my/experiment/ros.launch')
#         }
#         self.m_simconf.retina_config = 'retina_configuration'
#         self.m_simconf.ext_robot_controller = 'xyz.model'
#
#         MockUtil.fake_base(self, CLEGazeboSimulationAssembly)
#
#     def tearDown(self):
#         pass
#
#     def test_init(self):
#         import pydevd
#         pydevd.settrace('localhost', port=50001, stdoutToServer=True, stderrToServer=True)
#         self.assembly = CLEGazeboSimulationAssembly(self.m_simconf)
#         self.m_baseClass = super(CLEGazeboSimulationAssembly, self.assembly)
#
#         self.m_baseClass.__init__.assert_called_once_with(self.m_simconf)
#         self.assertTrue(self.assembly.cle_server is None)
#         self.assertTrue(self.assembly.tempAssetsDir is not None)
#         self.assertTrue(self.assembly.storage_client is not None)
#
#     def test_initialize(self):
#         self.assembly = CLEGazeboSimulationAssembly(self.m_simconf)
#         self.m_baseClass = (CLEGazeboSimulationAssembly, self)
#
#         self.assembly._initialize(lambda: None)
#
#         self.assertTrue(self.assembly.cle_server is not None)
#         self.m_server.setup_handlers.assert_called_once()
#         # test loading was called
#         self.m_baseClass._start_gazebo.assert_called_once()
#
#         self.m_baseClass.robotManager.scene_handler.return_value.load_textures.assert_called_once()
#         self.m_baseClass.robotManager.scene_handler.return_value\
#             .parse_gazebo_world_file.assert_called_once_with(
#             self.m_simconf.world_model.resource_path.abs_path)
#
#         self.m_baseClass.robotManager.set_robot_dict.assert_called_once_with(
#             self.m_simconf.robot_models)
#
#         self.assertTrue(self.m_baseClass.robotManager.retina_config is self.m_simconf.retina_config)
#
#         self.m_baseClass.robotManager.initialize.assert_called_once_with(
#             self.m_simconf.robot_models['bb8'])
#         self.m_baseClass.robotManager.initialize.assert_called_once_with(
#             self.m_simconf.robot_models['r2d2'])
#
#         self.m_subproc.call.assert_called_once_with(['/my/experiment/xyz.model', 'start'])
#
#     # def test_missing_texture(self):
#     #     self.m_storage.get_texture.side_effect = Exception
#     #     self.assembly._initialize(lambda: None)
#     #     # test loading never called
#     #     self.m_baseClass.robotManager.scene_handler.return_value.load_textures.assert_not_called()
#     #
#     # def test_exception_loading_texture(self):
#     #     self.m_baseClass.robotManager.scene_handler.side_effect = Exception
#     #     # test initialize goes through
#     #     try:
#     #         self.assembly._initialize(lambda: None)
#     #     except:
#     #         raise Exception("Initialize should not have failed")
#     #
#     # def test_properties(self):
#     #     pass
#
#
# if __name__ == '__main__':
#     unittest.main()
