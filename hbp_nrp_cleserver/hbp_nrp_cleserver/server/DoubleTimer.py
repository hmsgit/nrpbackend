"""
This module contains an implementation to run a certain function repeatedly until a timeout is
reached
"""

from threading import Thread, Event
import logging
import math

__author__ = 'Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli'

logger = logging.getLogger(__name__)


# from http://stackoverflow.com/questions/12435211/
#             python-threading-timer-repeat-function-every-n-seconds
class DoubleTimer(Thread):
    """
    Timer that runs two functions, one every n1 seconds and the other
    every n2 seconds, using only one thread
    """

    def __init__(self, interval1, callback1, interval2, callback2):
        """
        Construct the timer. To make it work properly, interval2 must be a
        multiple of interval1.

        :param interval1: the time interval of the first function
        :param callback1: the first function to be called
        :param interval2: the time interval of the second function
        :param callback2: the second function to be called
        """
        Thread.__init__(self)
        self.setDaemon(True)
        if interval1 <= 0 or interval2 <= 0:
            logger.error("interval1 or interval2 must be positive")
            raise ValueError("interval1 or interval2 must be positive")
        remainder = math.fmod(interval2, interval1)
        if remainder > 1e-10 and interval1 - remainder > 1e-10:
            logger.error("interval2 of Double timer is not a multiple \
                          of interval1")
            raise ValueError("interval2 is not a multiple of interval1")
        self.interval1 = interval1
        self.callback1 = callback1
        self.interval2 = interval2
        self.callback2 = callback2
        self.stopped = Event()
        self.stopped.clear()
        self.counter = 0
        self.expiring = False

    def run(self):
        """
        Exec the function and restart the timer.
        """
        while not self.stopped.wait(self.interval1):
            self.callback1()
            if self.expiring:
                self.counter += 1
                if self.counter >= self.interval2 / self.interval1:
                    self.counter = 0
                    self.expiring = False
                    self.callback2()

    def is_expiring(self):
        """
        Return True if the second callback is active, False otherwise.
        """
        return self.expiring

    def cancel_all(self):
        """
        Cancel the timer.
        """
        self.disable_second_callback()
        self.stopped.set()

    def enable_second_callback(self):
        """
        Enable calling of the second callback (one call only).
        """
        self.expiring = True

    def disable_second_callback(self):
        """
        Disable calling of the second callback and reset its timer.
        """
        self.expiring = False
        self.counter = 0

    def remaining_time(self):
        """
        Get remaining time before the second callback is executed.
        If the callback is disabled, the function will return the full
        interval2.

        :return: the remaining time before the callback
        """
        if not self.expiring:
            return self.interval2
        else:
            return self.interval2 - self.counter * self.interval1
