#!/usr/bin/env python
"""
This file simply acts as mock state machine script for the unit tests. The created state machine
is simply a generated mock instance
"""

__author__ = 'Sebastian Krach'

import mock

initialize_cb = None


def create_state_machine():
    """
    Returns a mock instance as state machine for this module

    :return: a mock instance
    """

    return mock_instance


def __initialize_side_effect(*args, **kwargs):
    global initialize_cb
    initialize_cb = args[0]


mock_instance = mock.Mock()
mock_instance.sm.register_termination_cb.side_effect = __initialize_side_effect









