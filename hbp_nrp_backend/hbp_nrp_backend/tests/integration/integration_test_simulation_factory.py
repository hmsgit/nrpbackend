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
