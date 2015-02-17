"""
This module contains a helper class for the REST services to identify and check the user
calling the service.
"""

__author__ = 'Oliver Denninger'

from flask_restful import reqparse
import logging

logger = logging.getLogger("__main__")


class UserAuthentication(object):
    """
    Helper class to get the user, authenticated at the HBP Unified Portal, from a HTTP request.
    This is done by reading the value of the 'X-User-Name' header. If this header is not present,
    the user name 'default-owner' ist used to allow testing outside the portal, etc.
    """

    HTTP_HEADER_USER_NAME = "X-User-Name"

    @staticmethod
    def get_x_user_name_header(request):
        """
        Gets the value of the 'X-User-Name' header from the given HTTP request
        :param request: The request
        :return: The value of the 'X-User-Name' header or if not found 'default-owner'
        """
        request_parser = reqparse.RequestParser()
        request_parser.add_argument(UserAuthentication.HTTP_HEADER_USER_NAME,
                                    type=str, location='headers')
        user = request_parser.parse_args(request)[UserAuthentication.HTTP_HEADER_USER_NAME]
        if user is None:
            user = "default-owner"

        return user

    @staticmethod
    def matches_x_user_name_header(request, user):
        """
        Checks if the value of the 'X-User-Name' header from the given HTTP request matches
        the given user name
        :param request: The request
        :param user: The user name to check
        :return: True if value of the 'X-User-Name' header and user name match, False otherwise
        """
        request_user = UserAuthentication.get_x_user_name_header(request)
        if user == request_user:
            return True
        else:
            logger.warn("Request from user '" + request_user + "' but simulation owned by '"
                        + user + "'")
            return False
