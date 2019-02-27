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
This module represents the configuration of a running simulation
"""

from geometry_msgs.msg import Pose
import tf.transformations as transformations


class SimConfUtil(object):
    """
    Utility functions for the SimConfig module
    """
    @staticmethod
    def convertXSDPosetoPyPose(xsd_pose):
        """
        Converts a xml DOM pose object to python Pose object

        :param xsd_pose: DOM object reference containing the pose
        :return: a converted Pose object
        """
        if xsd_pose is None:
            return None

        rpose = Pose()
        rpose.position.x = xsd_pose.x
        rpose.position.y = xsd_pose.y
        rpose.position.z = xsd_pose.z

        # REST request has no ux defined. Exc DOM however has one
        if getattr(xsd_pose, 'ux', None) is not None:  # pragma: no cover
            rpose.orientation.x = xsd_pose.ux
            rpose.orientation.y = xsd_pose.uy
            rpose.orientation.z = xsd_pose.uz
            rpose.orientation.w = xsd_pose.theta
        else:
            roll = xsd_pose.roll if xsd_pose.roll is not None else 0
            pitch = xsd_pose.pitch if xsd_pose.pitch is not None else 0
            yaw = xsd_pose.yaw if xsd_pose.yaw is not None else 0

            quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)

            rpose.orientation.x = quaternion[0]
            rpose.orientation.y = quaternion[1]
            rpose.orientation.z = quaternion[2]
            rpose.orientation.w = quaternion[3]

        if rpose.orientation.x is None or rpose.orientation.y is None or \
                rpose.orientation.z is None or rpose.orientation.w is None:  # pragma: no cover
            rpose.orientation.x = 0.0
            rpose.orientation.y = 0.0
            rpose.orientation.z = 0.0
            rpose.orientation.w = 1.0

        return rpose
