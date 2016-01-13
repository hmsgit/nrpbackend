"""
Takes care of the operations on the Collab portal (includes Collab and Document clients)
"""

from hbp_nrp_backend import NRPServicesGeneralException


class NRPServicesUploadException(NRPServicesGeneralException):
    """
    Exception class used to pass error message to the front-end
    in case a file upload fails

    :param message: message displayed to the end user.
    """
    def __init__(self, message, error_type="Upload error"):
        super(NRPServicesUploadException, self).__init__(message, error_type)
