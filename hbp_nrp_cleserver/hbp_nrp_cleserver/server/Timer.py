"""
This module contains an implementation to run a certain function repeatedly until a timeout is
reached
"""

from threading import Thread, Event
import logging

__author__ = 'Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli'

logger = logging.getLogger(__name__)


# from http://stackoverflow.com/questions/12435211/
#             python-threading-timer-repeat-function-every-n-seconds
class Timer(Thread):
    """
    Timer that runs two functions, one every n1 seconds and the other
    every n2 seconds, using only one thread
    """

    def __init__(self, interval, callback):
        """
        Construct the timer.

        :param interval: the time interval
        :param callback: the function to be called
        """
        Thread.__init__(self)
        self.setDaemon(True)
        self.interval = interval
        self.callback = callback
        self.stopped = Event()
        self.stopped.clear()

    def run(self):
        """
        Exec the function and restart the timer.
        """
        while not self.stopped.wait(self.interval):
            self.callback()

    def cancel_all(self):
        """
        Cancel the timer.
        """
        self.stopped.set()
