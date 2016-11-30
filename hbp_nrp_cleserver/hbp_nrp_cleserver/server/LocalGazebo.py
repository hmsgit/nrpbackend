"""
This module contains the classes needed to have all gazebo services running locally.
"""

from hbp_nrp_cleserver.server.GazeboInterface import IGazeboServerInstance, IGazeboBridgeInstance
from hbp_nrp_cleserver.server.Watchdog import Watchdog
import os
from hbp_nrp_cle import config
import logging

__author__ = 'Alessandro Ambrosano'

# Info messages sent to this logger will be forwarded as notifications
logger = logging.getLogger('hbp_nrp_cle.user_notifications')


class LocalGazeboServerInstance(IGazeboServerInstance):
    """
    Represents a local instance of gzserver.
    """
    def __init__(self):
        super(LocalGazeboServerInstance, self).__init__()
        self.__watchdog = None

    def start(self, ros_master_uri): # pylint: disable=unused-argument
        """
        Starts a gzserver instance connected to the local roscore (provided by
        ros_master_uri)

        :param: ros_master_uri The ros master uri where to connect gzserver.
        """
        logger.info("Starting gzserver")
        os.system(config.config.get('gazebo', 'restart-cmd'))
        self.__ensure_watchdog_running()

    def __ensure_watchdog_running(self):
        """
        Makes sure that a watchdog is running
        """
        if self.__watchdog is None:
            self.__watchdog = Watchdog("gzserver", self._raise_gazebo_died)
            self.__watchdog.start()

    def stop(self):
        """
        Stops the gzserver instance.
        """
        logger.info("Stopping gzserver")
        os.system(config.config.get('gazebo', 'stop-cmd'))
        if self.__watchdog is not None:
            self.__watchdog.stop()
            self.__watchdog = None

    def restart(self, ros_master_uri):
        """
        Restarts the gzserver instance.
        """
        logger.info("Restarting gzserver")
        os.system(config.config.get('gazebo', 'restart-cmd'))
        self.__ensure_watchdog_running()

    # pylint: disable=R0201
    def try_extend(self, new_timeout): # pylint: disable=unused-argument
        """
        Always accept new simulation timeout
        """

        return True

    @property
    def gazebo_master_uri(self):
        """
        Returns a string containing the gazebo master
        URI (like:'http://bbpviz001.cscs.ch:11345')
        """

        result = os.environ.get("GAZEBO_MASTER_URI")
        if (not result):
            # Default gazebo URI (when environment variable is empty)
            result = "http://localhost:11345"
        return result


class LocalGazeboBridgeInstance(IGazeboBridgeInstance):
    """
    Represents a local instance of gzbridge.
    """

    def start(self): # pylint: disable=unused-argument
        """
        Starts the gzbridge instance represented by the object.
        """
        logger.info("Starting gzbridge")
        os.system(config.config.get('gzbridge', 'start-cmd'))

    def stop(self):
        """
        Stops the gzbridge instance represented by the object.
        """
        logger.info("Stopping gzbridge")
        os.system(config.config.get('gzbridge', 'stop-cmd'))

    def restart(self): # pylint: disable=unused-argument
        """
        Restarts the gzbridge instance represented by the object.
        """
        logger.info("Restarting gzbridge")
        os.system(config.config.get('gzbridge', 'restart-cmd'))
