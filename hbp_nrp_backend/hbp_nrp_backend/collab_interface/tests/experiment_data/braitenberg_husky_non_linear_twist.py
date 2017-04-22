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
This module contains the transfer function which is responsible for determining the non-linear twist
component of the husky's movement based on the left and right wheel neuron
(used for unit tests only)
"""
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.robotsim.RobotInterface import Topic
import geometry_msgs.msg


@nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.actors[2], nrp.leaky_integrator_alpha)
@nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))
def non_linear_twist(t, left_wheel_neuron, right_wheel_neuron):
    """
    The transfer function which calculates the linear twist of the husky robot based on the
    voltage of left and right wheel neuron.

    :param t: the current simulation time
    :param left_wheel_neuron: the left wheel neuron device
    :param right_wheel_neuron: the right wheel neuron device
    :return: a geometry_msgs/Twist message setting the linear twist fo the husky robot movement.
    """

    return geometry_msgs.msg.Twist(
        linear=geometry_msgs.msg.Vector3(
            x=20.0 * min(left_wheel_neuron.voltage, right_wheel_neuron.voltage),
            y=0.0,
            z=0.0
        ), angular=geometry_msgs.msg.Vector3(
            x=0.0,
            y=0.0,
            z=100.0 * (right_wheel_neuron.voltage - left_wheel_neuron.voltage)) ** 2
        )