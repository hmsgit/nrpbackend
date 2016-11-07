"""
This module contains a small python program that checks the availability of a local program
"""

from hbp_nrp_cleserver.server.Timer import Timer
import psutil
import logging


__author__ = "Georg Hinkel"


logger = logging.getLogger(__name__)


class Watchdog(object):
    """
    This class implements a watchdog that regularly checks whether a given process is still alive
    """
    def __init__(self, process, callback, pid=None, interval=1):
        """
        Creates a watchdog to monitor whether the given process is still running

        :param process: The process name that should be watched for
        :param callback: The method pointer that should be called when the process disappeared
        :param pid: The pid of the watched process, if already known (otherwise looked up)
        :param interval: The interval in which the process should be polled
        """
        self.__pid = pid
        self.__process = process
        self.__callback = callback
        self.__timer = Timer(interval, self._watch)

        if pid is not None:
            try:
                p = psutil.Process(pid)
                if process not in p.name():
                    logger.warn("The given process id is not valid. The process with the given "
                                "name is called {0} and does not match the given pattern {1}."
                                .format(p.name(), process))
                    self.__pid = None
            except psutil.NoSuchProcess:
                logger.warn("The specified process id does not exist. Ignoring process id")
                self.__pid = None

    def _watch(self):
        """
        Watches the process
        """
        if not self._is_alive():
            self.__callback()

    def reset(self):
        """
        Resets the watched pid
        """
        self.__pid = None

    @property
    def pid(self):
        """
        The process id that is watched for
        """
        return self.__pid

    def start(self):
        """
        Starts the watchdog in a new thread
        """
        self.__timer.start()

    def stop(self):
        """
        Stops the watchdog
        """
        self.__timer.cancel_all()

    def _is_alive(self):
        """
        Determines whether the watched process is still alive

        :return:
        """
        if self.__pid is None:
            for p in psutil.process_iter():
                if self.__process in p.name():
                    self.__pid = p.pid
                    return True
            if self.__pid is None:
                logger.info("Process {0} could not be found".format(self.__process))
                return False
        return psutil.pid_exists(self.__pid)
