"""
This module handles status messages and error messages coming from the CLE
"""

__author__ = 'Georg Hinkel'


class ROSCLEHandler(object):
    """
    This class handles status messages and errors originating from the CLE
    """

    def handle_error(self, error):
        """
        Handles error messages from the CLE

        :param error: The error message
        """
        raise NotImplementedError("This method is abstract")

    def handle_status(self, status):
        """
        Handles status messages from the CLE

        :param status: The simulation status
        """
        raise NotImplementedError("This method is abstract")
