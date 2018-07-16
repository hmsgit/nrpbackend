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
The package contains the implementation of the Closed Loop Engine Server (CLEServer) and
the Close Loop Engine Simulation Factory (CLESimulationFactory)

The ROSCLEServer module provides utility classes to run the Closed Loop
Engine in a separate process, while communicating with it through ROS services.
"""
from hbp_nrp_cle import config

__author__ = 'LorenzoVannucci'

ROS_CLE_NODE_NAME = config.config.get('ros', 'ros-cle-node-name')
SERVICE_VERSION = '/%s/version' % ROS_CLE_NODE_NAME
SERVICE_HEALTH = '/%s/health' % ROS_CLE_NODE_NAME
TOPIC_STATUS = '/%s/status' % ROS_CLE_NODE_NAME
TOPIC_LIFECYCLE = lambda sim_id: '/%s/%d/lifecycle' % (ROS_CLE_NODE_NAME, sim_id)
TOPIC_CLE_ERROR = '/%s/cle_error' % ROS_CLE_NODE_NAME
SERVICE_CREATE_NEW_SIMULATION = '/%s/create_new_simulation' % ROS_CLE_NODE_NAME
SERVICE_IS_SIMULATION_RUNNING = '/%s/is_simulation_running' % ROS_CLE_NODE_NAME
SERVICE_SIM_START_ID = lambda sim_id: '/%s/%d/start' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_STOP_ID = lambda sim_id: '/%s/%d/stop' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_PAUSE_ID = lambda sim_id: '/%s/%d/pause' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_RESET_ID = lambda sim_id: '/%s/%d/reset' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_EXTEND_TIMEOUT_ID = lambda sim_id: '/%s/%d/extend_timeout' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_STATE_ID = lambda sim_id: '/%s/%d/state' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_TRANSFER_FUNCTIONS = lambda sim_id: \
    '/%s/%d/get_transfer_functions' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_ADD_TRANSFER_FUNCTION = lambda sim_id: \
    '/%s/%d/add_transfer_function' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_EDIT_TRANSFER_FUNCTION = lambda sim_id: \
    '/%s/%d/edit_transfer_function' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_ACTIVATE_TRANSFER_FUNCTION = lambda sim_id: \
    '/%s/%d/activate_transfer_function' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_DELETE_TRANSFER_FUNCTION = lambda sim_id: \
    '/%s/%d/delete_transfer_function' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_BRAIN = lambda sim_id: '/%s/%d/get_brain' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SET_BRAIN = lambda sim_id: '/%s/%d/set_brain' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_POPULATIONS = lambda sim_id: '/%s/%d/get_populations' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_CSV_RECORDERS_FILES = lambda sim_id:\
    '/%s/%d/get_CSV_recorders_files' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_CLEAN_CSV_RECORDERS_FILES = lambda sim_id:\
    '/%s/%d/clean_CSV_recorders_files' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIMULATION_RECORDER = lambda sim_id:\
    '/%s/%d/simulation_recorder' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_CONVERT_TRANSFER_FUNCTION_RAW_TO_STRUCTURED = lambda sim_id:\
    '/%s/%d/convert_transfer_function_raw_to_structured' % (ROS_CLE_NODE_NAME, sim_id)


def ros_handler(func):
    """
    A decorator for ROS handlers.

    :param func: the function to decorate.
    :return: a wrapped function
    """

    def inner(*args, **kwargs):
        """
        A function that wraps func returning [] when func returns None.
        """
        q = func(*args, **kwargs)
        return [] if q is None else q

    return inner
