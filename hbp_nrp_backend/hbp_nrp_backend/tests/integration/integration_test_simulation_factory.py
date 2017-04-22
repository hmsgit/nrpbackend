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
This module starts the ROSCLESimulationFactory pushing all exceptions to a dedicated ROS topic
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cleserver.server import ROSCLESimulationFactory
import rospy
import std_msgs.msg
import sys
import logging

error_publisher = None


def unhandled_exception(type, value, traceback):
    error_message = "Unhandled exception of type {0}: {1}".format(type, value)
    if error_publisher is not None:
        error_publisher.publish(error_message)

if __name__ == "__main__":
    sys.excepthook = unhandled_exception

    logging.root.name = "SimulationFactory"

    server = ROSCLESimulationFactory.ROSCLESimulationFactory()
    logging.info("Initialize CLE server")
    server.initialize()
    ROSCLESimulationFactory.set_up_logger(None)
    logging.info("Create publisher for exceptions")
    error_publisher = rospy.Publisher("/integration_test/exceptions",
                                      std_msgs.msg.String, queue_size=10)
    logging.info("Starting CLE server")
    server.run()
    logging.info("CLE server shutdown")
