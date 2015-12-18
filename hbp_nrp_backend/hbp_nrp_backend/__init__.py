"""
This package contains the python code to run the REST web server and supportive tooling
"""

from hbp_nrp_backend.version import VERSION as __version__  # pylint: disable=unused-import
from hbp_nrp_backend import config
import os

__author__ = 'GeorgHinkel'

setting_str = os.environ.get('APP_SETTINGS')
# The APP_SETTINGS environment variable allows you to easily switch configurations.
# Standard configurations are listed in config.py .
# APP_SETTINGS defaults to config.DeploymentConfig on a puppet-managed server
# (see nrp-services/nrp-services-env.sh from server-scripts repo).
# Some other valid config objects: config.TestConfig, config.LocalConfig.
if setting_str is not None:
    config_class_str = setting_str.split('.')[-1]
    # Application configuration object, shared accross all hbp_nrp_backend files
    hbp_nrp_backend_config = getattr(config, config_class_str)
else:
    hbp_nrp_backend_config = None


class NRPServicesGeneralException(Exception):
    """
    General exception class that can be used to return meaningful messages
    to the ExD frontend.

    :param message: message displayed to the end user.
    :param error_type: Type of error (like 'CLE Error')
    :param error_code: The HTTP error code to send to the frontend.
    """
    def __init__(self, message, error_type, error_code=500):
        super(NRPServicesGeneralException, self).__init__(message)
        # This field is handled by the error handling HBP frontend code.
        self.error_type = error_type
        self.error_code = error_code

    def __str__(self):
        return "{0} ({1})".format(repr(self.message), self.error_type)


class NRPServicesClientErrorException(NRPServicesGeneralException):
    """
    Exception class for client (4xx) errors. It can be used to return meaningful messages
    to the ExD frontend.

    :param message: message displayed to the end user.
    :param error_code: The HTTP error code to send to the frontend.
    """
    def __init__(self, message, error_type="Client error", error_code=400):
        super(NRPServicesClientErrorException, self).__init__(message, error_type, error_code)


class NRPServicesStateException(NRPServicesGeneralException):
    """
    State exception class that can be used to return meaningful messages
    to the HBP frontend code.

    :param message: message displayed to the end user.
    """
    def __init__(self, message):
        super(NRPServicesStateException, self).__init__(message, "Transition error", 400)


class NRPServicesTransferFunctionException(NRPServicesGeneralException):
    """
    Transfer function exception class that can be used to return meaningful messages
    to the HBP frontend in case source code updates fail.

    :param message: message displayed to the end user.
    """
    def __init__(self, message):
        super(NRPServicesTransferFunctionException, self).\
            __init__(message, "Transfer function error", 400)


class NRPServicesStateMachineException(NRPServicesClientErrorException):
    """
    State machine exception class that can be used to return meaningful messages
    to the HBP frontend in case source code updates fail.
    :param message: message displayed to the end user.
    """
    def __init__(self, message, error_code):
        super(NRPServicesStateMachineException, self).\
            __init__(message, "State machine error", error_code)


class NRPServicesWrongUserException(NRPServicesClientErrorException):
    """
    State machine exception class that can be used to return meaningful messages
    to the HBP frontend in case source code updates fail.
    :param message: message displayed to the end user.
    """
    def __init__(self):
        super(NRPServicesWrongUserException, self).\
            __init__(
                "You need to be the simulation owner to apply your changes",
                "Wrong user",
                401
            )


class NRPServicesUnavailableROSService(NRPServicesGeneralException):
    """
    Server error with status 500 (internal error) that a ROS service is unavailable.
    It can be used to return meaningful messages to the ExD frontend.

    :param message: message displayed to the end user. It contains the text of the
                    corresponding ROS exception
    """
    def __init__(self, message):
        super(NRPServicesUnavailableROSService, self).__init__(
            "ROS service not available: " + message,
            "Unavailable ROS service"
        )
