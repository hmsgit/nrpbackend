# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
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
This module handles textures loading
"""
import logging
from gazebo_msgs.srv import SpawnEntity, SpawnEntityRequest, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Point, Quaternion, Pose
import rospy
import tf
logger = logging.getLogger(__name__)


class TexturesLoader(object):
    """"
    Textures loader
    """
    # pylint: disable=no-self-use

    def load_textures(self, token, experiment_id):
        """
        Loads the user textures in the simulation

        :param token: The token of the request
        :param experiment_id: The experiment id
        """
        from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient
        client = StorageClient()
        textures = client.get_textures(experiment_id, token)
        if textures:
            texture_name = textures[0]['name']
            # We dynamically spawn a new box with no collision and transparent just to trigger
            # the gazebo material addition. Then the material can be used from a state machine
            # that sets the colour. When done we delete the temp object
            model = '<?xml version="1.0" ?>'                                                     \
                    + '<sdf version="1.5">'                                                      \
                    + ' <model name="custom_material">'                                          \
                    + '  <link name="link">'                                                     \
                    + '   <visual name="visual">'                                                \
                    + '    <geometry> '                                                          \
                    + '     <box>'                                                               \
                    + '      <size>0.001 0.001 0.001</size>'                                     \
                    + '     </box>'                                                              \
                    + '    </geometry>'                                                          \
                    + '    <material><script>'                                                   \
                    + '     <uri>file://custom_textures/materials/scripts/custom.material</uri>' \
                    + '     <name>' + texture_name + '</name>'                                   \
                    + '    </script></material>'                                                 \
                    + '    <transparency>1</transparency>'                                       \
                    + '   </visual>'                                                             \
                    + '  </link>'                                                                \
                    + ' </model>'                                                                \
                    + '</sdf>'
            qRot = tf.transformations.quaternion_from_euler(
                0, 0, 0)

            pose = Pose(Point(0, 0,
                              0), Quaternion(*qRot))
            try:
                # We wait maximum 5 seconds before we give up
                rospy.wait_for_service('/gazebo/spawn_sdf_entity', 5)
                service_proxy = rospy.ServiceProxy(
                    '/gazebo/spawn_sdf_entity', SpawnEntity)
                service_proxy.call(SpawnEntityRequest(texture_name,
                                                      model,
                                                      "",
                                                      pose,
                                                      'world'))
                rospy.wait_for_service('/gazebo/delete_model', 5)
                service_proxy = rospy.ServiceProxy(
                    '/gazebo/delete_model', DeleteModel)
                service_proxy.call(DeleteModelRequest(texture_name))

            except rospy.ROSException:
                message = "Timeout while trying to load textures."
                logger.info(message)
