# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains a helper class for the REST services to identify and check the user
calling the service.
"""

__author__ = 'Oliver Denninger'

from flask_restful import reqparse
from flask import request
import logging
from backports.functools_lru_cache import lru_cache
from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient

logger = logging.getLogger("__main__")


class UserAuthentication(object):
    """
    Helper class to get the user, authenticated at the HBP Unified Portal, from a HTTP request.
    This is done by reading the value of the 'X-User-Name' header. If this header is not present,
    the user name 'default-owner' ist used to allow testing outside the portal, etc.
    """

    HTTP_HEADER_USER_NAME = "X-User-Name"
    HEADER_TOKEN = "Authorization"
    DEFAULT_OWNER = "default-owner"
    NO_TOKEN = "no_token"
    client = StorageClient()

    @staticmethod
    def get_header(header_name, default_value):
        """
        Gets the value of the headername header from the current HTTP
        :param default_value: If nothing is found, this will be returned
        :return: The value of the headername header or if not found default_value
        """
        request_parser = reqparse.RequestParser()
        request_parser.add_argument(header_name,
                                    type=str, location='headers')
        header_value = request_parser.parse_args(request)[header_name]
        if header_value is None:
            header_value = default_value

        return header_value

    @staticmethod
    def get_x_user_name_header():
        """
        Gets the value of the 'X-User-Name' header from the current HTTP
        :return: The value of the 'X-User-Name' header or if not found 'default-owner'
        """
        return UserAuthentication.get_header(UserAuthentication.HTTP_HEADER_USER_NAME,
                                              UserAuthentication.DEFAULT_OWNER)

    @staticmethod
    def get_header_token():
        """
        Gets the value of the 'Authorization' header from the current HTTP
        :return: The value of the 'Authorization' header or if not found 'no-token'
        """
        token_field = UserAuthentication.get_header(UserAuthentication.HEADER_TOKEN,
                                                     UserAuthentication.NO_TOKEN)
        if (token_field != UserAuthentication.NO_TOKEN):
            return token_field.split()[1]
        else:
            return token_field

    @staticmethod
    @lru_cache()
    def get_token_owner(token):
        """
        Gets the owner of an authentication token
        :param token: The authentication token
        :return: The user's id
        """
        try:
            user = UserAuthentication.client.get_user(token)
            return user['id'] if user else None
        # pylint: disable=broad-except
        except Exception:
            return None

    @staticmethod
    def get_user():
        """
        Gets the owner of the current request, from the header data
        It uses the x-user-name if specified, else by the authentication token
        :return: The user's id
        """
        username = UserAuthentication.get_x_user_name_header()
        if username != UserAuthentication.DEFAULT_OWNER:
            return username

        token = UserAuthentication.get_header_token()
        if token == UserAuthentication.NO_TOKEN:
            return username

        token_owner = UserAuthentication.get_token_owner(token)
        return token_owner if token_owner else username

    @staticmethod
    @lru_cache()
    def __user_can_access_experiment(token, context_id, experiment_id):
        """
        Checkis if a user can access a simulation.
        :param token: The authentication token
        :param context_id: Optional context idenfifier
        :param experiment_id: The simulation's experiment id
        :return: Whether the user can access the simulation
        """

        if token == UserAuthentication.NO_TOKEN:
            return False

        try:
            return UserAuthentication.client.can_acess_experiment(token, context_id, experiment_id)
        # pylint: disable=broad-except
        except Exception:
            return False

    @staticmethod
    def can_view(simulation):
        """
        Checks if the current HTTP request is authenticated by
        someone the simulation was shared with
        :param simulation: The simulation
        :return: True if the simulation was shared with the request's user, False otherwise
        """

        token = UserAuthentication.get_header_token()
        return UserAuthentication.__user_can_access_experiment(token,
                                                               simulation.ctx_id,
                                                               simulation.experiment_id)

    @staticmethod
    def can_modify(simulation):
        """
        Checks if the current HTTP request is authenticated by the simulation owner
        :param simulation: The simulation
        :return: True if the request was triggered by a simulation owner, False otherwise
        """
        user = simulation.owner
        request_user = UserAuthentication.get_user()
        if user == request_user:
            return True
        else:
            logger.warn("Request from user '{request_user}' but simulation owned by '{user}'"
                        .format(request_user=request_user, user=user))
            return False
