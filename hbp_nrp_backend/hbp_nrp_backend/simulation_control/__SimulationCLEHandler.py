"""
This module handles status messages and error messages coming from the CLE
"""

from hbp_nrp_backend.cle_interface.ROSCLEHandler import ROSCLEHandler

__author__ = 'Georg Hinkel'


class SimulationCLEHandler(ROSCLEHandler):
    """
    This class handles status messages and errors originating from the CLE for a specific simulation
    """

    def __init__(self, simulation):
        super(SimulationCLEHandler, self).__init__()

        self.__simulation = simulation

    def handle_error(self, error):
        """
        Handles error messages from the CLE

        :param error: The error message
        """
        self.__simulation.state = "halted"

    def handle_status(self, status):
        """
        Handles status messages from the CLE

        :param status: The simulation status
        """
        if status.status == "failed":
            self.__simulation.state = "halted"
