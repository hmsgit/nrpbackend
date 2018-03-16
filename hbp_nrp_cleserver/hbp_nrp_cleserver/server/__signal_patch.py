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
This module patches signal such that signal handlers can be registered from other threads
"""

import signal


handlers = {}


def __signal_handler(sig, frame):
    """
    This function is used as the default signal handler

    :param sig: The signal that has been received
    :param frame: The current application frame
    """
    if sig in handlers:
        for handler in handlers[sig]:
            handler(sig, frame)


def __new_signal(sig, handler):
    """
    Registers the given signal handler

    :param sig: The signal that should be reacted to
    :param handler: The handler for the signal
    """
    if sig not in handlers:
        handlers[sig] = []
    handlers[sig].append(handler)


def patch_signal():
    """
    Patches the signal module to allow signal handler registration from other threads
    """
    signal.signal(signal.SIGINT, __signal_handler)
    signal.signal(signal.SIGTERM, __signal_handler)
    signal.signal = __new_signal
