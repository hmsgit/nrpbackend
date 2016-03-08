"""
This package contains the interface to CLE, allowing the backend to control the simulation.
"""


ROS_CLE_NODE_NAME = 'ros_cle_simulation'  # Duplicate of ros_cle_simulation in config.ini

# Duplicate of the variables in hbp_nrp_cle.cle_interface.__init__
SERVICE_CREATE_NEW_SIMULATION = '/%s/create_new_simulation' % (ROS_CLE_NODE_NAME, )
SERVICE_SIM_START_ID = lambda sim_id: '/%s/%d/start' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_STOP_ID = lambda sim_id: '/%s/%d/stop' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_PAUSE_ID = lambda sim_id: '/%s/%d/pause' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_RESET_ID = lambda sim_id: '/%s/%d/reset' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_STATE_ID = lambda sim_id: '/%s/%d/state' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_TRANSFER_FUNCTIONS = lambda sim_id: \
    '/%s/%d/get_transfer_functions' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SET_TRANSFER_FUNCTION = lambda sim_id: \
    '/%s/%d/set_transfer_function' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_DELETE_TRANSFER_FUNCTION = lambda sim_id: \
    '/%s/%d/delete_transfer_function' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_BRAIN = lambda sim_id: '/%s/%d/get_brain' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SET_BRAIN = lambda sim_id: '/%s/%d/set_brain' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_POPULATIONS = lambda sim_id: '/%s/%d/get_populations' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_CSV_RECORDERS_FILES = lambda sim_id:\
    '/%s/%d/get_CSV_recorders_files' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_CLEAN_CSV_RECORDERS_FILES = lambda sim_id:\
    '/%s/%d/clean_CSV_recorders_files' % (ROS_CLE_NODE_NAME, sim_id)

TOPIC_STATUS = '/%s/status' % ROS_CLE_NODE_NAME
TOPIC_CLE_ERROR = '/%s/cle_error' % ROS_CLE_NODE_NAME
