"""
This package contains the interface to CLE, allowing the backend to control the simulation.
"""


ROS_CLE_NODE_NAME = 'ros_cle_simulation'  # Duplicate of ros_cle_simulation in config.ini

# Duplicate of the variables in hbp_nrp_cle.cle_interface.__init__
SERVICE_START_NEW_SIMULATION = '/%s/start_new_simulation' % (ROS_CLE_NODE_NAME, )
SERVICE_SIM_START_ID = lambda sim_id: '/%s/%d/start' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_STOP_ID = lambda sim_id: '/%s/%d/stop' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_PAUSE_ID = lambda sim_id: '/%s/%d/pause' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_RESET_ID = lambda sim_id: '/%s/%d/reset' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_SIM_STATE_ID = lambda sim_id: '/%s/%d/state' % (ROS_CLE_NODE_NAME, sim_id)
SERVICE_GET_TRANSFER_FUNCTIONS = lambda sim_id: \
                                 '/%s/%d/get_transfer_functions' % (ROS_CLE_NODE_NAME, sim_id)
